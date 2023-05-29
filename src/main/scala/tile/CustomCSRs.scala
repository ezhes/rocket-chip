// See LICENSE.SiFive for license details.

package freechips.rocketchip.tile

import chisel3._
import chisel3.util.MixedVec
import org.chipsalliance.cde.config.Parameters

case class CustomCSR(id: Int, mask: BigInt, init: Option[BigInt], hasWritePort:Boolean = false)

object CustomCSR {
  def constant(id: Int, value: BigInt): CustomCSR = CustomCSR(id, BigInt(0), Some(value))
}

class CustomCSRIO(implicit p: Parameters) extends CoreBundle {
  val wen = Output(Bool())
  val wdata = Output(UInt(xLen.W))
  val value = Output(UInt(xLen.W))
}

class CustomCSRIOWritable(implicit p: Parameters) extends CustomCSRIO {
   /*
   In order to support CSRs which change their value due to unprompted action
   (such as an unrelated, non-CSRRW event happening elsewhere), it is desirable
   to have custom CSRs which have an extra write port on them in addition to the
   one available for use through the regular CSRRW flow. For example, an
   accelerator may wish to push additional status information into a CSR if it
   triggers an exception 
   */

   // Is the CSR File attempting to write to this CSR due to a regular write 
   // request? I.e. a CSRRW instruction is being committed
   val csrf_wen = Output(Bool())
   // Should the extra write port be enabled? Note that in the case that both
   // rf_wen and wport_wen are true, wport always wins. Thus, it is the 
   // the responsibility of wport_wdata's driver to appropriately respond to 
   // cases where the implementation is attempting to write to a CSR while a
   // standard CSRF write is occurring.
   val wport_wen = Input(Bool())
   val wport_wdata = Input(UInt(xLen.W))
 }


class CustomCSRs(implicit p: Parameters) extends CoreBundle {
  // Not all cores have these CSRs, but those that do should follow the same
  // numbering conventions.  So we list them here but default them to None.
  protected def bpmCSRId = 0x7c0
  protected def bpmCSR: Option[CustomCSR] = None

  protected def chickenCSRId = 0x7c1
  protected def chickenCSR: Option[CustomCSR] = None

  // If you override this, you'll want to concatenate super.decls
  def decls: Seq[CustomCSR] = bpmCSR.toSeq ++ chickenCSR

   /* 
   csrs needs to be lazy so that subclasses can override decls. If non-lazy,
   csrs reads only our private decls as we get init'd before the derived class.
   */
   lazy val csrs = {
     MixedVec(decls.map( csr =>
       if (csr.hasWritePort) {
         new CustomCSRIOWritable
       } else {
         new CustomCSRIO
       }
     ))
   }
  def flushBTB = getOrElse(bpmCSR, _.wen, false.B)
  def bpmStatic = getOrElse(bpmCSR, _.value(0), false.B)
  def disableDCacheClockGate = getOrElse(chickenCSR, _.value(0), false.B)
  def disableICacheClockGate = getOrElse(chickenCSR, _.value(1), false.B)
  def disableCoreClockGate = getOrElse(chickenCSR, _.value(2), false.B)
  def disableSpeculativeICacheRefill = getOrElse(chickenCSR, _.value(3), false.B)
  def suppressCorruptOnGrantData = getOrElse(chickenCSR, _.value(9), false.B)

  protected def getByIdOrElse[T](id: Int, f: CustomCSRIO => T, alt: T): T = {
    val idx = decls.indexWhere(_.id == id)
    if (idx < 0) alt else f(csrs(idx))
  }

  protected def getOrElse[T](csr: Option[CustomCSR], f: CustomCSRIO => T, alt: T): T =
    csr.map(c => getByIdOrElse(c.id, f, alt)).getOrElse(alt)
}
