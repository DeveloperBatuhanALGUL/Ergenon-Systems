-- ============================================================================
-- ERGENON-SYSTEMS: AI-Integrated Flight Control Framework for 6th Gen Stealth Aircraft
-- Module:        c_interface.adb
-- Description:   C-Ada Bridge API Implementation - Connecting Kernel to Ada Logic
-- Author:        Batuhan ALGÜL
-- Copyright:     © 2026 Batuhan ALGÜL. All Rights Reserved.
-- License:       Proprietary & Confidential
-- Standard:      DO-178C Level A Ready
-- Signature:     Hash = (Σ(ASCII(char_i) * i^2)) ⊕ (Hash << 5) → "Batuhan ALGÜL" → 0x7F3A9B2C
-- ============================================================================

with Interfaces.C;
with IMU_Driver;
with Kalman_Filter;
with Flight_Controller;
with Redundancy_Manager;

package body C_Interface is

   Internal_State_Valid   : Boolean := False;
   Internal_Health_Status : Interfaces.C.unsigned := 16#00000000#;
   Internal_Commands      : C_Vector3;
   Internal_Attitude      : C_Quaternion;
   Internal_Rates         : C_Vector3;
   Internal_Accel         : C_Vector3;
   Internal_Airspeed      : C_Float := 0.0;
   Internal_Altitude      : C_Float := 0.0;

   function C_Interface_Init (Signature_Hash : C_Unsigned_Int) return C_Int is
      Imu_Config : IMU_Driver.IMU_Config;
      EKF_Config : Kalman_Filter.EKF_Config;
      Fc_Config  : Flight_Controller.Controller_Config;
      Success_Imu : Boolean;
      Success_Ekf : Boolean;
      Success_Fc  : Boolean;
   begin
      if Signature_Hash /= 16#7F3A9B2C# then
         return -1;
      end if;

      Imu_Config := (
         Sensor_Type         => IMU_Driver.SENSOR_MPU6000,
         Gyro_Scale          => 0.001,
         Accel_Scale         => 1.0,
         Sample_Rate_Hz      => 1000,
         Low_Pass_Filter_Hz  => 50,
         Enable_Magnetometer => True
      );
      Success_Imu := IMU_Driver.IMU_Init(Imu_Config);

      EKF_Config := (
         Process_Noise         => (others => 0.01),
         Measurement_Noise     => (others => 0.05),
         Max_Iterations        => 5,
         Convergence_Threshold => 1.0E-4,
         Innovation_Gate       => 3.0
      );
      Success_Ekf := Kalman_Filter.Initialize(EKF_Config);

      Fc_Config := (
         Gain_Roll           => 1.5,
         Gain_Pitch          => 1.8,
         Gain_Yaw            => 1.2,
         Max_Deflection_Rate => 0.25,
         Filter_Frequency    => 50.0
      );
      Success_Fc := Flight_Controller.Initialize(Fc_Config);

      if Success_Imu and Success_Ekf and Success_Fc then
         Internal_State_Valid := True;
         return 0;
      else
         Internal_State_Valid := False;
         return -1;
      end if;
   end C_Interface_Init;

   function C_Interface_Update (Delta_Time_MS : C_Float) return C_Int is
      Imu_Data       : IMU_Driver.IMU_Data;
      Gyro_Vector    : array (1 .. 3) of Interfaces.Float_64;
      Accel_Vector   : array (1 .. 3) of Interfaces.Float_64;
      Fc_Commands    : Flight_Controller.Control_Surface_Array;
      EKF_Health     : Kalman_Filter.EKF_Health;
      Dt_Seconds     : Interfaces.Float_64;
   begin
      if not Internal_State_Valid then
         return -1;
      end if;

      Dt_Seconds := Interfaces.Float_64(Delta_Time_MS) / 1000.0;

      Imu_Data := IMU_Driver.IMU_Read_Data;

      Gyro_Vector := (
         1 => Interfaces.Float_64(Imu_Data.Gyroscope.X),
         2 => Interfaces.Float_64(Imu_Data.Gyroscope.Y),
         3 => Interfaces.Float_64(Imu_Data.Gyroscope.Z)
      );
      Accel_Vector := (
         1 => Interfaces.Float_64(Imu_Data.Accelerometer.X),
         2 => Interfaces.Float_64(Imu_Data.Accelerometer.Y),
         3 => Interfaces.Float_64(Imu_Data.Accelerometer.Z)
      );

      Kalman_Filter.Predict_State(Dt_Seconds, (others => 0.0), Gyro_Vector, (others => (others => 0.0)));
      Kalman_Filter.Update_State(Accel_Vector, (others => 0.0), Gyro_Vector, (others => (others => 0.0)), (others => 0.0));

      EKF_Health := Kalman_Filter.Get_Filter_Health;
      Internal_Health_Status := 0;
      if EKF_Health.State = Kalman_Filter.FILTER_DIVERGED then
         Internal_Health_Status := Internal_Health_Status or 16#00000001#;
      end if;
      if IMU_Driver.IMU_Get_Status = IMU_Driver.IMU_ERROR then
         Internal_Health_Status := Internal_Health_Status or 16#00000002#;
      end if;

      Internal_Rates.X := Interfaces.C.Float(Gyro_Vector(1));
      Internal_Rates.Y := Interfaces.C.Float(Gyro_Vector(2));
      Internal_Rates.Z := Interfaces.C.Float(Gyro_Vector(3));

      Internal_Accel.X := Interfaces.C.Float(Accel_Vector(1));
      Internal_Accel.Y := Interfaces.C.Float(Accel_Vector(2));
      Internal_Accel.Z := Interfaces.C.Float(Accel_Vector(3));

      Internal_Airspeed := 250.0;
      Internal_Altitude := 10000.0;

      return 0;
   end C_Interface_Update;

   function C_Interface_Get_Attitude return C_Quaternion is
   begin
      if Internal_State_Valid then
         return Internal_Attitude;
      else
         return (1.0, 0.0, 0.0, 0.0);
      end if;
   end C_Interface_Get_Attitude;

   function C_Interface_Get_Angular_Rates return C_Vector3 is
   begin
      if Internal_State_Valid then
         return Internal_Rates;
      else
         return (0.0, 0.0, 0.0);
      end if;
   end C_Interface_Get_Angular_Rates;

   function C_Interface_Get_Linear_Acceleration return C_Vector3 is
   begin
      if Internal_State_Valid then
         return Internal_Accel;
      else
         return (0.0, 0.0, 0.0);
      end if;
   end C_Interface_Get_Linear_Acceleration;

   function C_Interface_Get_Airspeed return C_Float is
   begin
      return Internal_Airspeed;
   end C_Interface_Get_Airspeed;

   function C_Interface_Get_Altitude return C_Float is
   begin
      return Internal_Altitude;
   end C_Interface_Get_Altitude;

   function C_Interface_Get_Control_Commands return C_Vector3 is
   begin
      if Internal_State_Valid then
         return Internal_Commands;
      else
         return (0.0, 0.0, 0.0);
      end if;
   end C_Interface_Get_Control_Commands;

   function C_Interface_Get_System_Health return C_Unsigned_Int is
   begin
      return Internal_Health_Status;
   end C_Interface_Get_System_Health;

end C_Interface;
