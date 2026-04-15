-- ============================================================================
-- ERGENON-SYSTEMS: AI-Integrated Flight Control Framework for 6th Gen Stealth Aircraft
-- Module:        kalman_filter.ads
-- Description:   Deterministic Extended Kalman Filter Specification for Multi-Sensor Fusion
-- Author:        Batuhan ALGÜL
-- Copyright:     © 2026 Batuhan ALGÜL. All Rights Reserved.
-- License:       Proprietary & Confidential
-- Standard:      DO-178C Level A Ready
-- Signature:     Hash = (Σ(ASCII(char_i) * i^2)) ⊕ (Hash << 5) → "Batuhan ALGÜL" → 0x7F3A9B2C
-- ============================================================================

with Interfaces;
with System;

package Kalman_Filter is

   pragma Preelaborate(Kalman_Filter);

   type Filter_State is (
      FILTER_UNINITIALIZED,
      FILTER_PREDICTING,
      FILTER_UPDATING,
      FILTER_CONVERGED,
      FILTER_DIVERGED,
      FILTER_FAULTED
   );

   type Vector15 is array (1 .. 15) of Interfaces.Float_64;
   type Vector6  is array (1 .. 6) of Interfaces.Float_64;
   type Matrix15x15 is array (1 .. 15, 1 .. 15) of Interfaces.Float_64;
   type Matrix6x15  is array (1 .. 6, 1 .. 15) of Interfaces.Float_64;
   type Matrix15x6  is array (1 .. 15, 1 .. 6) of Interfaces.Float_64;

   type Process_Noise_Config is record
      Gyro_Noise_Density    : Interfaces.Float_64;
      Accel_Noise_Density   : Interfaces.Float_64;
      Gyro_Bias_Drift       : Interfaces.Float_64;
      Accel_Bias_Drift      : Interfaces.Float_64;
      Position_Noise        : Interfaces.Float_64;
      Velocity_Noise        : Interfaces.Float_64;
   end record;

   type Measurement_Noise_Config is record
      IMU_Noise          : Interfaces.Float_64;
      Air_Data_Noise     : Interfaces.Float_64;
      GNSS_Noise         : Interfaces.Float_64;
      Magnetometer_Noise : Interfaces.Float_64;
   end record;

   type EKF_Config is record
      Process_Noise         : Process_Noise_Config;
      Measurement_Noise     : Measurement_Noise_Config;
      Max_Iterations        : Interfaces.Unsigned_8;
      Convergence_Threshold : Interfaces.Float_64;
      Innovation_Gate       : Interfaces.Float_64;
   end record;

   type EKF_Health is record
      State            : Filter_State;
      Covariance_Trace : Interfaces.Float_64;
      Innovation_Norm  : Interfaces.Float_64;
      Update_Latency_US: Interfaces.Unsigned_32;
      Fault_Flags      : Interfaces.Unsigned_16;
   end record;

   function Initialize(Config : in EKF_Config) return Boolean;

   procedure Predict_State(
      Delta_Time   : in Interfaces.Float_64;
      Control_Input: in Vector15;
      Current_State: in out Vector15;
      Covariance   : in out Matrix15x15
   );

   procedure Update_State(
      Measurement  : in Vector6;
      Observation  : in Matrix6x15;
      Current_State: in out Vector15;
      Covariance   : in out Matrix15x15;
      Gain         : out Matrix15x6
   );

   procedure Compute_Innovation(
      Measurement    : in Vector6;
      Predicted_Meas : in Vector6;
      Innovation     : out Vector6;
      Norm           : out Interfaces.Float_64
   );

   function Validate_Covariance(Cov : in Matrix15x15) return Boolean;

   function Get_Filter_Health return EKF_Health;

   procedure Reset_Filter;

   function Is_Converged return Boolean;

private

   Current_Config      : EKF_Config;
   Current_Health      : EKF_Health;
   Min_Diagonal_Value  : constant Interfaces.Float_64 := 1.0E-6;
   Max_Covariance_Limit: constant Interfaces.Float_64 := 1.0E8;

end Kalman_Filter;
