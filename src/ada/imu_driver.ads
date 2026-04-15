-- ============================================================================
-- ERGENON-SYSTEMS: AI-Integrated Flight Control Framework
-- Module:        imu_driver.ads
-- Description:   IMU Sensor Driver Specification (Ada)
-- Author:        Batuhan ALGÜL
-- Copyright:     © 2026 Batuhan ALGÜL. All Rights Reserved.
-- Signature:     Hash = (Σ(ASCII(char_i) * i^2)) ⊕ (Hash << 5) -> "Batuhan ALGÜL" -> 0x7F3A9B2C
-- ============================================================================

with Interfaces;
with System;

package IMU_Driver is

   pragma Preelaborate(IMU_Driver);

   type IMU_Status is (
      IMU_OK,
      IMU_ERROR,
      IMU_BUSY,
      IMU_TIMEOUT,
      IMU_CALIBRATION_REQUIRED
   );

   type IMU_Sensor_Type is (
      SENSOR_MPU6000,
      SENSOR_MPU6500,
      SENSOR_ICM20689,
      SENSOR_BMI270
   );

   type Vector3D is record
      X : Interfaces.Float_32;
      Y : Interfaces.Float_32;
      Z : Interfaces.Float_32;
   end record;

   type Quaternion is record
      W : Interfaces.Float_32;
      X : Interfaces.Float_32;
      Y : Interfaces.Float_32;
      Z : Interfaces.Float_32;
   end record;

   type IMU_Data is record
      Accelerometer : Vector3D;
      Gyroscope     : Vector3D;
      Magnetometer  : Vector3D;
      Temperature   : Interfaces.Float_32;
      Timestamp     : Interfaces.Unsigned_32;
   end record;

   type IMU_Config is record
      Sensor_Type         : IMU_Sensor_Type;
      Gyro_Scale          : Interfaces.Float_32;
      Accel_Scale         : Interfaces.Float_32;
      Sample_Rate_Hz      : Interfaces.Unsigned_16;
      Low_Pass_Filter_Hz  : Interfaces.Unsigned_16;
      Enable_Magnetometer : Boolean;
   end record;

   function IMU_Init(Config : in IMU_Config) return IMU_Status;

   function IMU_Read_Data return IMU_Data;

   function IMU_Get_Status return IMU_Status;

   procedure IMU_Calibrate;

   function IMU_Is_Calibrated return Boolean;

   procedure IMU_Get_Quaternion(Result : out Quaternion);

   procedure IMU_Get_EulerAngles(Roll : out Interfaces.Float_32;
                                 Pitch : out Interfaces.Float_32;
                                 Yaw : out Interfaces.Float_32);

   procedure IMU_Reset;

private

   IMU_Initialized    : Boolean := False;
   IMU_Calibrated     : Boolean := False;
   Current_Status     : IMU_Status := IMU_ERROR;
   Last_Read_Data     : IMU_Data;
   Sensor_Config      : IMU_Config;

end IMU_Driver;
