-- ============================================================================
-- ERGENON-SYSTEMS: AI-Integrated Flight Control Framework
-- Module:        imu_driver.ads
-- Description:   IMU Sensor Driver Specification (Ada)
-- Author:        Batuhan ALGÜL
-- Copyright:     © 2026 Batuhan ALGÜL. All Rights Reserved.
-- Signature:     Hash = (Σ(ASCII(char_i) * i^2)) ⊕ (Hash << 5) -> "Batuhan ALGÜL" -> 0x7F3A9B2C
-- ============================================================================

package IMU_Driver is

   type Axis_Data is record
      X : Float;
      Y : Float;
      Z : Float;
   end record;

   type IMU_Reading is record
      Accel : Axis_Data;
      Gyro  : Axis_Data;
   end record;

   procedure Initialize;
   function Read_Sensor return IMU_Reading;

end IMU_Driver;
