package freechips.rocketchip.rocket

import chisel3._
import chisel3.util._
import freechips.rocketchip.util._
import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.tile._
import freechips.rocketchip.util._

class NormalPhysicalAddress(implicit p: Parameters)
    extends CoreBundle()(p) with HasRocketCoreParameters {
    val address = UInt(width = coreMaxAddrBits.W)

    def getTagStoragePhysicalAddress(context:MTEManagerIO) = {
        val tspa = Wire(Valid(new TagStoragePhysicalAddress()(p)))

        // val tagIndex = address >> log2Ceil(MTEConfig.taggingGranuleBytes)
        // val byteIndex = tagIndex >> log2Ceil(8 / MTEConfig.tagBits)
        // require(MTEConfig.tagBits <= 8, "TCache assumes tags are a byte or less")
        /*
        For a 16-byte granule and 4-bit tag:
        [3 : 0] granule offset
        [4] sub-byte tag sel
        [: 5] byte region offset

        Generically:
        [granuleBytes2 - 1 : 0] granule offset
        [mteTagsPerByteBits + granuleBytes2 - 1: granuleBytes2] sub-byte tag sel
        [: 1 + mteTagsPerByteBits - 1 + granuleBytes2] byte region offset
        */
        val mteTagsPerByte = 8 / MTEConfig.tagBits
        val granuleBytes2 = log2Ceil(MTEConfig.taggingGranuleBytes)
        val subByteTagSelectIdxBegin = granuleBytes2
        val subByteTagSelectIdxEnd = log2Ceil(mteTagsPerByte) + subByteTagSelectIdxBegin - 1
        val byteRegionOffsetIdxBegin = subByteTagSelectIdxEnd + 1
        val byteRegionOffsetIdxEnd = coreMaxAddrBits - subByteTagSelectIdxEnd - 1

        tspa.bits.subByteTagSelect := address(subByteTagSelectIdxEnd, subByteTagSelectIdxBegin)
        
        val regionPack = rocketParams.mteRegions zip (context.cpu.mteRegionBases zip context.cpu.mteRegionMasks)
        val tagStorageAddress = Mux1H(regionPack.map {case (region, (storageBase, storageMask)) =>
            /* Generate the storage address */
            val regionByteOffset = address(byteRegionOffsetIdxEnd, byteRegionOffsetIdxBegin) - (region.base.U)(byteRegionOffsetIdxEnd, byteRegionOffsetIdxBegin)
            val storageByteAddress = storageBase + regionByteOffset

            /* Is the actual address contained by the tag region? */
            val inBounds = address >= region.base.U && address < (region.base + region.size).U && 
                /* 
                Is the storage address in bounds? 
                We check this by verifying the upper bits of the new address
                matches the upper bits of the original base (i.e. we didn't over
                flow the mask)
                */
                (storageByteAddress & ~storageMask) === (storageBase & ~storageMask) &&
                /* 
                Is the storage mask non-zero? A zero mask indicates a tagging is 
                disabled for the region 
                */
                storageMask =/= 0.U

            val v = Wire(Valid(UInt(coreMaxAddrBits.W)))
            v.valid := inBounds
            v.bits := storageByteAddress
            inBounds -> v
        })

        /* 
        We're valid if we have a valid hit. It's a SoC design error to build 
        something with overlapping regions so we're also for sure 1H
        */
        tspa.valid := tagStorageAddress.valid 
        tspa.bits.address := tagStorageAddress.bits

        tspa
    }
}

class TagStoragePhysicalAddress(implicit p: Parameters)
    extends CoreBundle()(p) {
    val address = UInt(width = coreMaxAddrBits.W)
    /** 
     * `address` refers to the byte aligned storage address. Since tags are 
     * sub-byte, we need to know which tag in the byte to select
     */  
    val subByteTagSelect = UInt(width = log2Ceil(8 / MTEConfig.tagBits).W)
    
    // def addressOffset = address(cacheOffsetOff + cacheOffsetBits - 1, cacheOffsetOff)

    // /** The index of this tag inside the data array line */
    // def addressMTETagIndex = Cat(addressOffset, subByteTagSelect)
}

object MTEOperationType extends ChiselEnum {
    val LOAD = Value
    val STORE = Value
    val TAG_WRITE = Value
}

class MTEManagerOperation(implicit p: Parameters) 
  extends CoreBundle()(p) {
  val paddr = UInt(coreMaxAddrBits.W)
  val op_type = MTEOperationType()
  val mem_size = UInt(2.W)
  val address_tag = UInt(MTEConfig.tagBits.W)
}

class MTEFaultPacket(implicit p: Parameters) 
  extends CoreBundle()(p) {
  val faulting_address = UInt(coreMaxAddrBits.W)
  val op_type = MTEOperationType()
  val mem_size = UInt(2.W)
  val physical_tag = UInt(MTEConfig.tagBits.W)
  val address_tag = UInt(MTEConfig.tagBits.W)
}

class MTEManagerCPUIO(implicit p: Parameters)
  extends CoreBundle()(p) with HasRocketCoreParameters {
  val req = Flipped(DecoupledIO(new MTEManagerOperation()(p)))
  val fault = Output(Valid(new MTEFaultPacket))
  val mteRegionBases = Input(Vec(rocketParams.mteRegions.size, UInt(width = coreMaxAddrBits.W)))
  val mteRegionMasks = Input(Vec(rocketParams.mteRegions.size, UInt(width = coreMaxAddrBits.W)))
  val mtePermissiveTag = Input(UInt(MTEConfig.tagBits.W))
  val mteEnabled = Input(Bool())
}

class MTEManagerIO(implicit p: Parameters) 
  extends CoreBundle()(p) with HasRocketCoreParameters {
  val cpu = new MTEManagerCPUIO()(p)
  val mem = new HellaCacheIO
}

/*
Requests are submitted to the manager after the request completes. That is, once
they are non-speculative. This doesn't really have any serious performance issues
since tag enforcement is entirely async.
*/

object MTEManagerState extends ChiselEnum {
    /** Ready to accept a new request */
    val READY = Value
    /** Waiting for downstream memory be ready to accept a request */
    val LINE_READ = Value
    val LINE_WAIT1 = Value
    val LINE_WAIT2 = Value
}

class MTEManager(implicit p: Parameters) 
  extends CoreModule()(p) with HasRocketCoreParameters {
  val io = new MTEManagerIO
  dontTouch(io)

  val fetchQueueDeq = Queue(io.cpu.req, rocketParams.mteTagFetchQueueDepth, flow = true)
  val next_state = Wire(MTEManagerState())
  next_state := DontCare
  val state = RegInit(MTEManagerState.READY)
  state := next_state

  val op = Reg(new MTEManagerOperation()(p))
  val op_tspa = Reg(new TagStoragePhysicalAddress()(p))

  /* Pop an operation from the queue and decode it */
  val op_will_be_valid = WireInit(false.B)
  when (state === MTEManagerState.READY) {
    op := fetchQueueDeq.bits
    
    val npa = Wire(new NormalPhysicalAddress()(p))
    npa.address := fetchQueueDeq.bits.paddr
    val tspa = npa.getTagStoragePhysicalAddress(io) 
    op_tspa := tspa.bits

    /* 
    Only accept an op if we actually deq'd something, it decodes to a valid tag
    storage location, and we are actually checking tags at the moment. 
    Otherwise, we just swallow it and don't bother checking for faults.
    */
    op_will_be_valid := fetchQueueDeq.valid && tspa.valid && io.cpu.mteEnabled
    when (fetchQueueDeq.valid && !tspa.valid) {
      printf("[mte] rejecting paddr=%x, tspa invalid\n", fetchQueueDeq.bits.paddr)
    }
  }
  fetchQueueDeq.ready := state === MTEManagerState.READY
  dontTouch(op_will_be_valid)
  dontTouch(next_state)

  io.mem.req.valid := state === MTEManagerState.LINE_READ ||
    state === MTEManagerState.LINE_WAIT2 && io.mem.s2_nack
  io.mem.s1_kill := false.B
  io.mem.s2_kill := false.B
  io.mem.s1_data.data := DontCare
  io.mem.s1_data.mask := Fill(coreDataBytes, 0.U(1.W))
  val mem_reqb = io.mem.req.bits
  mem_reqb.phys := true.B
  mem_reqb.size := 0.U
  mem_reqb.signed := false.B
  mem_reqb.dv := false.B
  mem_reqb.dprv := PRV.M.U
  mem_reqb.cmd := M_XRD
  // val align_mask = (coreDataBytes - 1).U(coreMaxAddrBits.W)
  // val masked_addr = op_tspa.address & ~align_mask
  // mem_reqb.addr := masked_addr
  mem_reqb.addr := op_tspa.address
  when (state === MTEManagerState.LINE_READ) {
    printf("[mte] fire! paddr=%x, tspa=%x\n",  op.paddr, op_tspa.address)
  }

  val mem_tags = Wire(Vec(8 / MTEConfig.tagBits, UInt(MTEConfig.tagBits.W)))
  mem_tags := io.mem.resp.bits.data.asTypeOf(mem_tags)
  val mem_tag = mem_tags(op_tspa.subByteTagSelect)

  /* We always update the fault record */
  val faultb = io.cpu.fault.bits
  faultb := DontCare
  io.cpu.fault.valid := false.B

  when (state === MTEManagerState.LINE_WAIT2 && !io.mem.s2_nack) {
    /* Data arrived! If we got nack'd, we're retrying */
    when (op.op_type =/= MTEOperationType.TAG_WRITE &&
          op.address_tag =/= mem_tag &&
          mem_tag =/= io.cpu.mtePermissiveTag) {
      /* Fault! Update the record */
      io.cpu.fault.valid := true.B
      faultb.faulting_address := op.paddr
      faultb.op_type := op.op_type
      faultb.mem_size := op.mem_size
      faultb.physical_tag := mem_tag
      faultb.address_tag := op.address_tag
    }
    printf("[mte] read paddr=%x, tspa=%x, data=%x\n",  op.paddr, op_tspa.address, io.mem.resp.bits.data)
  }

  switch (state) {
    is (MTEManagerState.READY) {
      when (op_will_be_valid) {
        next_state := MTEManagerState.LINE_READ
      } .otherwise {
        next_state := MTEManagerState.READY
      }
    }

    is (MTEManagerState.LINE_READ) {
      when (io.mem.req.fire) {
        /* Tag request fired, now we need to wait for it to come back */
        next_state := MTEManagerState.LINE_WAIT1
      } .otherwise {
        /* better luck next time */
        next_state := MTEManagerState.LINE_READ
      }
    }

    is (MTEManagerState.LINE_WAIT1) {
      next_state := MTEManagerState.LINE_WAIT2
    }

    is (MTEManagerState.LINE_WAIT2) {
      when (io.mem.s2_nack) {
        /* Failed, we're retry */
        when (io.mem.req.fire) {
          /* Tag request fired, now we need to wait for it to come back */
          next_state := MTEManagerState.LINE_WAIT1
        } .otherwise {
          /* We couldn't fire, go to retry wait */
          next_state := MTEManagerState.LINE_READ
        }
      } .otherwise {
        /* Data came back, we're done! */
        next_state := MTEManagerState.READY
      }
    }
  }

}