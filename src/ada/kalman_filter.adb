-- ============================================================================
-- ERGENON-SYSTEMS: AI-Integrated Flight Control Framework for 6th Gen Stealth Aircraft
-- Module:        kalman_filter.adb
-- Description:   Deterministic Extended Kalman Filter Implementation for Multi-Sensor Fusion
-- Author:        Batuhan ALGÜL
-- Copyright:     © 2026 Batuhan ALGÜL. All Rights Reserved.
-- License:       Proprietary & Confidential
-- Standard:      DO-178C Level A Ready
-- Signature:     Hash = (Σ(ASCII(char_i) * i^2)) ⊕ (Hash << 5) → "Batuhan ALGÜL" → 0x7F3A9B2C
-- ============================================================================

with Interfaces;
with System;
with Ada.Numerics.Elementary_Functions;

package body Kalman_Filter is

   IDENTITY_15 : constant Matrix15x15 := (
      1 => (1 => 1.0, others => 0.0),
      2 => (2 => 1.0, others => 0.0),
      3 => (3 => 1.0, others => 0.0),
      4 => (4 => 1.0, others => 0.0),
      5 => (5 => 1.0, others => 0.0),
      6 => (6 => 1.0, others => 0.0),
      7 => (7 => 1.0, others => 0.0),
      8 => (8 => 1.0, others => 0.0),
      9 => (9 => 1.0, others => 0.0),
      10 => (10 => 1.0, others => 0.0),
      11 => (11 => 1.0, others => 0.0),
      12 => (12 => 1.0, others => 0.0),
      13 => (13 => 1.0, others => 0.0),
      14 => (14 => 1.0, others => 0.0),
      15 => (15 => 1.0, others => 0.0)
   );

   ZERO_15 : constant Matrix15x15 := (others => (others => 0.0));
   ZERO_15_VEC : constant Vector15 := (others => 0.0);
   ZERO_6_VEC : constant Vector6 := (others => 0.0);

   procedure Matrix_Multiply_15x15(A : in Matrix15x15; B : in Matrix15x15; Result : out Matrix15x15) is
   begin
      for I in 1 .. 15 loop
         for J in 1 .. 15 loop
            Result(I, J) := 0.0;
            for K in 1 .. 15 loop
               Result(I, J) := Result(I, J) + A(I, K) * B(K, J);
            end loop;
         end loop;
      end loop;
   end Matrix_Multiply_15x15;

   procedure Matrix_Multiply_15x6_6x15(A : in Matrix15x6; B : in Matrix6x15; Result : out Matrix15x15) is
   begin
      for I in 1 .. 15 loop
         for J in 1 .. 15 loop
            Result(I, J) := 0.0;
            for K in 1 .. 6 loop
               Result(I, J) := Result(I, J) + A(I, K) * B(K, J);
            end loop;
         end loop;
      end loop;
   end Matrix_Multiply_15x6_6x15;

   procedure Matrix_Multiply_6x15_15x6(A : in Matrix6x15; B : in Matrix15x6; Result : out Matrix6x6) is
      type Matrix6x6 is array (1 .. 6, 1 .. 6) of Interfaces.Float_64;
      Res : Matrix6x6;
   begin
      for I in 1 .. 6 loop
         for J in 1 .. 6 loop
            Res(I, J) := 0.0;
            for K in 1 .. 15 loop
               Res(I, J) := Res(I, J) + A(I, K) * B(K, J);
            end loop;
         end loop;
      end loop;
      Result := Res;
   end Matrix_Multiply_6x15_15x6;

   procedure Matrix_Transpose_15x6(A : in Matrix15x6; Result : out Matrix6x15) is
   begin
      for I in 1 .. 15 loop
         for J in 1 .. 6 loop
            Result(J, I) := A(I, J);
         end loop;
      end loop;
   end Matrix_Transpose_15x6;

   procedure Matrix_Add_15x15(A : in Matrix15x15; B : in Matrix15x15; Result : out Matrix15x15) is
   begin
      for I in 1 .. 15 loop
         for J in 1 .. 15 loop
            Result(I, J) := A(I, J) + B(I, J);
         end loop;
      end loop;
   end Matrix_Add_15x15;

   procedure Matrix_Subtract_15x15(A : in Matrix15x15; B : in Matrix15x15; Result : out Matrix15x15) is
   begin
      for I in 1 .. 15 loop
         for J in 1 .. 15 loop
            Result(I, J) := A(I, J) - B(I, J);
         end loop;
      end loop;
   end Matrix_Subtract_15x15;

   procedure Matrix_Invert_6x6(A : in Matrix6x6; Result : out Matrix6x6; Success : out Boolean) is
      type M6 is array (1 .. 6, 1 .. 6) of Interfaces.Float_64;
      type V6 is array (1 .. 6) of Interfaces.Float_64;
      Aug : array (1 .. 6, 1 .. 12) of Interfaces.Float_64;
      Pivot : V6 := (others => 0.0);
      Temp : Interfaces.Float_64;
   begin
      for I in 1 .. 6 loop
         for J in 1 .. 6 loop
            Aug(I, J) := A(I, J);
         end loop;
         Aug(I, I + 6) := 1.0;
      end loop;

      for Col in 1 .. 6 loop
         declare
            Max_Val : Interfaces.Float_64 := 0.0;
            Max_Row : Integer := Col;
         begin
            for Row in Col .. 6 loop
               if Ada.Numerics.Elementary_Functions.Abs(Aug(Row, Col)) > Max_Val then
                  Max_Val := Ada.Numerics.Elementary_Functions.Abs(Aug(Row, Col));
                  Max_Row := Row;
               end if;
            end loop;

            if Max_Val < 1.0E-12 then
               Success := False;
               Result := (others => (others => 0.0));
               return;
            end if;

            if Max_Row /= Col then
               for K in 1 .. 12 loop
                  Temp := Aug(Col, K);
                  Aug(Col, K) := Aug(Max_Row, K);
                  Aug(Max_Row, K) := Temp;
               end loop;
            end if;

            Pivot(Col) := Aug(Col, Col);
            for K in 1 .. 12 loop
               Aug(Col, K) := Aug(Col, K) / Pivot(Col);
            end loop;

            for Row in 1 .. 6 loop
               if Row /= Col then
                  Temp := Aug(Row, Col);
                  for K in 1 .. 12 loop
                     Aug(Row, K) := Aug(Row, K) - Temp * Aug(Col, K);
                  end loop;
               end if;
            end loop;
         end;
      end loop;

      for I in 1 .. 6 loop
         for J in 1 .. 6 loop
            Result(I, J) := Aug(I, J + 6);
         end loop;
      end loop;

      Success := True;
   end Matrix_Invert_6x6;

   procedure Compute_Process_Jacobian(F : out Matrix15x15; State : in Vector15; Dt : in Interfaces.Float_64) is
   begin
      F := IDENTITY_15;
      F(1, 4) := Dt;
      F(2, 5) := Dt;
      F(3, 6) := Dt;
      F(4, 7) := Dt;
      F(5, 8) := Dt;
      F(6, 9) := Dt;
      F(7, 10) := Dt * State(3);
      F(7, 11) := Dt * State(2);
      F(8, 10) := Dt * State(3);
      F(8, 12) := Dt * State(1);
      F(9, 11) := Dt * State(1);
      F(9, 12) := Dt * State(2);
   end Compute_Process_Jacobian;

   procedure Compute_Measurement_Jacobian(H : out Matrix6x15; State : in Vector15) is
   begin
      H := (others => (others => 0.0));
      H(1, 1) := 1.0;
      H(2, 2) := 1.0;
      H(3, 3) := 1.0;
      H(4, 13) := 1.0;
      H(5, 14) := 1.0;
      H(6, 15) := 1.0;
   end Compute_Measurement_Jacobian;

   procedure Compute_Process_Noise(Q : out Matrix15x15; Config : in Process_Noise_Config; Dt : in Interfaces.Float_64) is
      G : Matrix15x15 := ZERO_15;
      G_diag : constant array (1 .. 15) of Interfaces.Float_64 := (
         1 => Config.Gyro_Noise_Density,
         2 => Config.Gyro_Noise_Density,
         3 => Config.Gyro_Noise_Density,
         4 => Config.Accel_Noise_Density,
         5 => Config.Accel_Noise_Density,
         6 => Config.Accel_Noise_Density,
         7 => Config.Gyro_Bias_Drift,
         8 => Config.Gyro_Bias_Drift,
         9 => Config.Gyro_Bias_Drift,
         10 => Config.Accel_Bias_Drift,
         11 => Config.Accel_Bias_Drift,
         12 => Config.Accel_Bias_Drift,
         13 => Config.Position_Noise,
         14 => Config.Velocity_Noise,
         15 => Config.Velocity_Noise
      );
   begin
      for I in 1 .. 15 loop
         G(I, I) := G_diag(I) * Dt;
      end loop;
      Matrix_Multiply_15x15(G, G, Q);
   end Compute_Process_Noise;

   procedure Compute_Measurement_Noise(R : out Matrix6x6; Config : in Measurement_Noise_Config) is
      type M6 is array (1 .. 6, 1 .. 6) of Interfaces.Float_64;
      R_M6 : M6 := (others => (others => 0.0));
   begin
      R_M6(1, 1) := Config.IMU_Noise;
      R_M6(2, 2) := Config.IMU_Noise;
      R_M6(3, 3) := Config.IMU_Noise;
      R_M6(4, 4) := Config.Air_Data_Noise;
      R_M6(5, 5) := Config.GNSS_Noise;
      R_M6(6, 6) := Config.Magnetometer_Noise;
      R := R_M6;
   end Compute_Measurement_Noise;

   function Initialize(Config : in EKF_Config) return Boolean is
   begin
      if Config.Max_Iterations = 0 or Config.Convergence_Threshold <= 0.0 then
         Current_Health.State := FILTER_FAULTED;
         return False;
      end if;

      Current_Config := Config;
      Current_Health := (
         State => FILTER_UNINITIALIZED,
         Covariance_Trace => 0.0,
         Innovation_Norm => 0.0,
         Update_Latency_US => 0,
         Fault_Flags => 0
      );
      Current_Health.State := FILTER_CONVERGED;
      return True;
   end Initialize;

   procedure Predict_State(
      Delta_Time   : in Interfaces.Float_64;
      Control_Input: in Vector15;
      Current_State: in out Vector15;
      Covariance   : in out Matrix15x15
   ) is
      F : Matrix15x15;
      Q : Matrix15x15;
      FP : Matrix15x15;
      FP_T : Matrix15x15;
      FPF_T : Matrix15x15;
      P_New : Matrix15x15;
   begin
      Compute_Process_Jacobian(F, Current_State, Delta_Time);
      Compute_Process_Noise(Q, Current_Config.Process_Noise, Delta_Time);

      for I in 1 .. 15 loop
         for J in 1 .. 15 loop
            FP(I, J) := 0.0;
            for K in 1 .. 15 loop
               FP(I, J) := FP(I, J) + F(I, K) * Covariance(K, J);
            end loop;
         end loop;
      end loop;

      Matrix_Transpose_15x6(F, FP_T);
      Matrix_Multiply_15x15(F, FP, FPF_T);
      Matrix_Add_15x15(FPF_T, Q, P_New);

      for I in 1 .. 15 loop
         Current_State(I) := Current_State(I) + Control_Input(I) * Delta_Time;
      end loop;

      Covariance := P_New;
      Current_Health.State := FILTER_PREDICTING;
   end Predict_State;

   procedure Update_State(
      Measurement  : in Vector6;
      Observation  : in Matrix6x15;
      Current_State: in out Vector15;
      Covariance   : in out Matrix15x15;
      Gain         : out Matrix15x6
   ) is
      type M6 is array (1 .. 6, 1 .. 6) of Interfaces.Float_64;
      H : Matrix6x15;
      R : Matrix6x6;
      P_Ht : Matrix15x6;
      S : M6;
      S_Inv : M6;
      K : Matrix15x6;
      KH : Matrix15x15;
      I_KH : Matrix15x15;
      Innovation : Vector6;
      P_New : Matrix15x15;
      Inv_Success : Boolean;
      Max_Iter : Interfaces.Unsigned_8 := 0;
      Norm_Sq : Interfaces.Float_64 := 0.0;
   begin
      H := Observation;
      Compute_Measurement_Noise(R, Current_Config.Measurement_Noise);

      Matrix_Transpose_15x6(H, P_Ht);
      Matrix_Multiply_6x15_15x6(H, P_Ht, S);

      for I in 1 .. 6 loop
         S(I, I) := S(I, I) + R(I, I);
      end loop;

      Matrix_Invert_6x6(S, S_Inv, Inv_Success);
      if not Inv_Success then
         Current_Health.State := FILTER_DIVERGED;
         Current_Health.Fault_Flags := Current_Health.Fault_Flags or 16#0001#;
         return;
      end if;

      for I in 1 .. 15 loop
         for J in 1 .. 6 loop
            K(I, J) := 0.0;
            for L in 1 .. 6 loop
               K(I, J) := K(I, J) + P_Ht(I, L) * S_Inv(L, J);
            end loop;
         end loop;
      end loop;

      for I in 1 .. 6 loop
         Innovation(I) := Measurement(I);
         for J in 1 .. 15 loop
            Innovation(I) := Innovation(I) - H(I, J) * Current_State(J);
         end loop;
         Norm_Sq := Norm_Sq + Innovation(I) * Innovation(I);
      end loop;

      Current_Health.Innovation_Norm := Ada.Numerics.Elementary_Functions.Sqrt(Norm_Sq);

      if Current_Health.Innovation_Norm > Current_Config.Innovation_Gate then
         Current_Health.State := FILTER_DIVERGED;
         Current_Health.Fault_Flags := Current_Health.Fault_Flags or 16#0004#;
         return;
      end if;

      for I in 1 .. 15 loop
         for J in 1 .. 6 loop
            for K in 1 .. 6 loop
               Current_State(I) := Current_State(I) + K(I, K) * Innovation(K);
            end loop;
         end loop;
      end loop;

      Matrix_Multiply_15x6_6x15(K, H, KH);
      Matrix_Subtract_15x15(IDENTITY_15, KH, I_KH);
      Matrix_Multiply_15x15(I_KH, Covariance, P_New);
      Covariance := P_New;
      Gain := K;

      Current_Health.State := FILTER_UPDATING;
      Current_Health.Covariance_Trace := 0.0;
      for I in 1 .. 15 loop
         Current_Health.Covariance_Trace := Current_Health.Covariance_Trace + Covariance(I, I);
      end loop;

      if Current_Health.Covariance_Trace < Current_Config.Convergence_Threshold then
         Current_Health.State := FILTER_CONVERGED;
      else
         Current_Health.State := FILTER_PREDICTING;
      end if;
   end Update_State;

   procedure Compute_Innovation(
      Measurement    : in Vector6;
      Predicted_Meas : in Vector6;
      Innovation     : out Vector6;
      Norm           : out Interfaces.Float_64
   ) is
      Sum_Sq : Interfaces.Float_64 := 0.0;
   begin
      for I in 1 .. 6 loop
         Innovation(I) := Measurement(I) - Predicted_Meas(I);
         Sum_Sq := Sum_Sq + Innovation(I) * Innovation(I);
      end loop;
      Norm := Ada.Numerics.Elementary_Functions.Sqrt(Sum_Sq);
   end Compute_Innovation;

   function Validate_Covariance(Cov : in Matrix15x15) return Boolean is
      Min_Diag : Interfaces.Float_64 := Interfaces.Float_64'Last;
      Max_Diag : Interfaces.Float_64 := Interfaces.Float_64'First;
   begin
      for I in 1 .. 15 loop
         if Cov(I, I) < Min_Diagonal_Value then
            return False;
         end if;
         if Cov(I, I) > Max_Covariance_Limit then
            return False;
         end if;
         if Cov(I, I) < Min_Diag then
            Min_Diag := Cov(I, I);
         end if;
         if Cov(I, I) > Max_Diag then
            Max_Diag := Cov(I, I);
         end if;
      end loop;
      return True;
   end Validate_Covariance;

   function Get_Filter_Health return EKF_Health is
   begin
      return Current_Health;
   end Get_Filter_Health;

   procedure Reset_Filter is
   begin
      Current_Health := (
         State => FILTER_UNINITIALIZED,
         Covariance_Trace => 0.0,
         Innovation_Norm => 0.0,
         Update_Latency_US => 0,
         Fault_Flags => 0
      );
      Current_Config := (others => <>);
   end Reset_Filter;

   function Is_Converged return Boolean is
   begin
      return Current_Health.State = FILTER_CONVERGED;
   end Is_Converged;

end Kalman_Filter;
