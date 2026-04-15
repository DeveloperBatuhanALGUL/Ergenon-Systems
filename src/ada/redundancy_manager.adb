-- ============================================================================
-- ERGENON-SYSTEMS: AI-Integrated Flight Control Framework for 6th Gen Stealth Aircraft
-- Module:        redundancy_manager.adb
-- Description:   Triple-Modular Redundancy (TMR) and Voting Logic Implementation
-- Author:        Batuhan ALGÜL
-- Copyright:     © 2026 Batuhan ALGÜL. All Rights Reserved.
-- License:       Proprietary & Confidential
-- Standard:      DO-178C Level A Ready
-- Signature:     Hash = (Σ(ASCII(char_i) * i^2)) ⊕ (Hash << 5) → "Batuhan ALGÜL" → 0x7F3A9B2C
-- ============================================================================

with Interfaces;
with System;
with Ada.Numerics.Elementary_Functions;

package body Redundancy_Manager is

   function Min_3(A : Interfaces.Float_32; B : Interfaces.Float_32; C : Interfaces.Float_32) return Interfaces.Float_32 is
      Min_Val : Interfaces.Float_32 := A;
   begin
      if B < Min_Val then
         Min_Val := B;
      end if;
      if C < Min_Val then
         Min_Val := C;
      end if;
      return Min_Val;
   end Min_3;

   function Max_3(A : Interfaces.Float_32; B : Interfaces.Float_32; C : Interfaces.Float_32) return Interfaces.Float_32 is
      Max_Val : Interfaces.Float_32 := A;
   begin
      if B > Max_Val then
         Max_Val := B;
      end if;
      if C > Max_Val then
         Max_Val := C;
      end if;
      return Max_Val;
   end Max_3;

   function Mid_Value_Select(A : Interfaces.Float_32; B : Interfaces.Float_32; C : Interfaces.Float_32) return Interfaces.Float_32 is
      Sum : Interfaces.Float_32 := A + B + C;
      Minimum : Interfaces.Float_32 := Min_3(A, B, C);
      Maximum : Interfaces.Float_32 := Max_3(A, B, C);
   begin
      return Sum - Minimum - Maximum;
   end Mid_Value_Select;

   procedure Compute_Voted_Signal(
      Sig_Left  : in Interfaces.Float_32;
      Sig_Center: in Interfaces.Float_32;
      Sig_Right : in Interfaces.Float_32;
      Voted_Out : out Interfaces.Float_32;
      Max_Diff  : out Interfaces.Float_32
   ) is
      Diff_LC : Interfaces.Float_32;
      Diff_CR : Interfaces.Float_32;
      Diff_RL : Interfaces.Float_32;
   begin
      Diff_LC := Ada.Numerics.Elementary_Functions.Abs(Sig_Left - Sig_Center);
      Diff_CR := Ada.Numerics.Elementary_Functions.Abs(Sig_Center - Sig_Right);
      Diff_RL := Ada.Numerics.Elementary_Functions.Abs(Sig_Right - Sig_Left);

      Max_Diff := Diff_LC;
      if Diff_CR > Max_Diff then
         Max_Diff := Diff_CR;
      end if;
      if Diff_RL > Max_Diff then
         Max_Diff := Diff_RL;
      end if;

      case Current_Voting_Mode is
         when MODE_MID_VALUE_SELECT =>
            Voted_Out := Mid_Value_Select(Sig_Left, Sig_Center, Sig_Right);
         when MODE_AVERAGE =>
            Voted_Out := (Sig_Left + Sig_Center + Sig_Right) / 3.0;
         when MODE_MAJORITY_VOTE =>
            if Diff_LC < Threshold_Airspeed then
               Voted_Out := (Sig_Left + Sig_Center) / 2.0;
            elsif Diff_CR < Threshold_Airspeed then
               Voted_Out := (Sig_Center + Sig_Right) / 2.0;
            else
               Voted_Out := Sig_Center;
            end if;
      end case;
   end Compute_Voted_Signal;

   function Initialize(Voting_Method : in Voting_Mode) return Boolean is
   begin
      Current_Voting_Mode := Voting_Method;
      System_Operational := True;
      Channel_States := (others => HEALTH_OK);
      return True;
   end Initialize;

   procedure Update_Channel_Health(
      ID     : in Channel_ID;
      Status : in Channel_Health
   ) is
   begin
      Channel_States(ID) := Status;
   end Update_Channel_Health;

   procedure Input_Channel_Data(
      ID   : in Channel_ID;
      Data : in Channel_Signal_Data
   ) is
   begin
      Channel_Inputs(ID) := Data;
   end Input_Channel_Data;

   function Compute_Voted_Signals return Voted_Output is
      Result : Voted_Output;
      Diff_Roll      : Interfaces.Float_32;
      Diff_Pitch     : Interfaces.Float_32;
      Diff_Yaw       : Interfaces.Float_32;
      Diff_Airspeed  : Interfaces.Float_32;
      Diff_Altitude  : Interfaces.Float_32;
      Valid_Count    : Interfaces.Unsigned_8 := 0;
   begin
      Result.Valid := False;
      Result.Active_Channels := 0;

      if Channel_States(CHANNEL_LEFT) = HEALTH_OK then
         Valid_Count := Valid_Count + 1;
      end if;
      if Channel_States(CHANNEL_CENTER) = HEALTH_OK then
         Valid_Count := Valid_Count + 1;
      end if;
      if Channel_States(CHANNEL_RIGHT) = HEALTH_OK then
         Valid_Count := Valid_Count + 1;
      end if;

      Result.Active_Channels := Valid_Count;

      if Valid_Count = 0 then
         Result.Roll_Rate := 0.0;
         Result.Pitch_Rate := 0.0;
         Result.Yaw_Rate := 0.0;
         Result.Vertical_Accel := 0.0;
         Result.Lateral_Accel := 0.0;
         Result.Longitudinal_Accel := 0.0;
         Result.Airspeed := 0.0;
         Result.Altitude := 0.0;
         return Result;
      elsif Valid_Count = 1 then
         if Channel_States(CHANNEL_LEFT) = HEALTH_OK then
            Result := (
               Roll_Rate => Channel_Inputs(CHANNEL_LEFT).Roll_Rate,
               Pitch_Rate => Channel_Inputs(CHANNEL_LEFT).Pitch_Rate,
               Yaw_Rate => Channel_Inputs(CHANNEL_LEFT).Yaw_Rate,
               Vertical_Accel => Channel_Inputs(CHANNEL_LEFT).Vertical_Accel,
               Lateral_Accel => Channel_Inputs(CHANNEL_LEFT).Lateral_Accel,
               Longitudinal_Accel => Channel_Inputs(CHANNEL_LEFT).Longitudinal_Accel,
               Airspeed => Channel_Inputs(CHANNEL_LEFT).Airspeed,
               Altitude => Channel_Inputs(CHANNEL_LEFT).Altitude,
               Valid => True,
               Active_Channels => 1
            );
         elsif Channel_States(CHANNEL_CENTER) = HEALTH_OK then
            Result := (
               Roll_Rate => Channel_Inputs(CHANNEL_CENTER).Roll_Rate,
               Pitch_Rate => Channel_Inputs(CHANNEL_CENTER).Pitch_Rate,
               Yaw_Rate => Channel_Inputs(CHANNEL_CENTER).Yaw_Rate,
               Vertical_Accel => Channel_Inputs(CHANNEL_CENTER).Vertical_Accel,
               Lateral_Accel => Channel_Inputs(CHANNEL_CENTER).Lateral_Accel,
               Longitudinal_Accel => Channel_Inputs(CHANNEL_CENTER).Longitudinal_Accel,
               Airspeed => Channel_Inputs(CHANNEL_CENTER).Airspeed,
               Altitude => Channel_Inputs(CHANNEL_CENTER).Altitude,
               Valid => True,
               Active_Channels => 1
            );
         else
            Result := (
               Roll_Rate => Channel_Inputs(CHANNEL_RIGHT).Roll_Rate,
               Pitch_Rate => Channel_Inputs(CHANNEL_RIGHT).Pitch_Rate,
               Yaw_Rate => Channel_Inputs(CHANNEL_RIGHT).Yaw_Rate,
               Vertical_Accel => Channel_Inputs(CHANNEL_RIGHT).Vertical_Accel,
               Lateral_Accel => Channel_Inputs(CHANNEL_RIGHT).Lateral_Accel,
               Longitudinal_Accel => Channel_Inputs(CHANNEL_RIGHT).Longitudinal_Accel,
               Airspeed => Channel_Inputs(CHANNEL_RIGHT).Airspeed,
               Altitude => Channel_Inputs(CHANNEL_RIGHT).Altitude,
               Valid => True,
               Active_Channels => 1
            );
         end if;
         return Result;
      else
         Compute_Voted_Signal(
            Channel_Inputs(CHANNEL_LEFT).Roll_Rate,
            Channel_Inputs(CHANNEL_CENTER).Roll_Rate,
            Channel_Inputs(CHANNEL_RIGHT).Roll_Rate,
            Result.Roll_Rate,
            Diff_Roll
         );

         Compute_Voted_Signal(
            Channel_Inputs(CHANNEL_LEFT).Pitch_Rate,
            Channel_Inputs(CHANNEL_CENTER).Pitch_Rate,
            Channel_Inputs(CHANNEL_RIGHT).Pitch_Rate,
            Result.Pitch_Rate,
            Diff_Pitch
         );

         Compute_Voted_Signal(
            Channel_Inputs(CHANNEL_LEFT).Yaw_Rate,
            Channel_Inputs(CHANNEL_CENTER).Yaw_Rate,
            Channel_Inputs(CHANNEL_RIGHT).Yaw_Rate,
            Result.Yaw_Rate,
            Diff_Yaw
         );

         Compute_Voted_Signal(
            Channel_Inputs(CHANNEL_LEFT).Vertical_Accel,
            Channel_Inputs(CHANNEL_CENTER).Vertical_Accel,
            Channel_Inputs(CHANNEL_RIGHT).Vertical_Accel,
            Result.Vertical_Accel,
            Diff_Airspeed
         );

         Compute_Voted_Signal(
            Channel_Inputs(CHANNEL_LEFT).Lateral_Accel,
            Channel_Inputs(CHANNEL_CENTER).Lateral_Accel,
            Channel_Inputs(CHANNEL_RIGHT).Lateral_Accel,
            Result.Lateral_Accel,
            Diff_Airspeed
         );

         Compute_Voted_Signal(
            Channel_Inputs(CHANNEL_LEFT).Longitudinal_Accel,
            Channel_Inputs(CHANNEL_CENTER).Longitudinal_Accel,
            Channel_Inputs(CHANNEL_RIGHT).Longitudinal_Accel,
            Result.Longitudinal_Accel,
            Diff_Airspeed
         );

         Compute_Voted_Signal(
            Channel_Inputs(CHANNEL_LEFT).Airspeed,
            Channel_Inputs(CHANNEL_CENTER).Airspeed,
            Channel_Inputs(CHANNEL_RIGHT).Airspeed,
            Result.Airspeed,
            Diff_Airspeed
         );

         Compute_Voted_Signal(
            Channel_Inputs(CHANNEL_LEFT).Altitude,
            Channel_Inputs(CHANNEL_CENTER).Altitude,
            Channel_Inputs(CHANNEL_RIGHT).Altitude,
            Result.Altitude,
            Diff_Altitude
         );

         Result.Valid := True;
         return Result;
      end if;
   end Compute_Voted_Signals;

   function Get_Channel_Health(ID : in Channel_ID) return Channel_Health is
   begin
      return Channel_States(ID);
   end Get_Channel_Health;

   function Get_Discrepancy_Report return Discrepancy_Info is
      Report : Discrepancy_Info;
   begin
      Report.Max_Diff_Roll := Ada.Numerics.Elementary_Functions.Abs(
         Channel_Inputs(CHANNEL_LEFT).Roll_Rate - Channel_Inputs(CHANNEL_CENTER).Roll_Rate
      );
      Report.Max_Diff_Pitch := Ada.Numerics.Elementary_Functions.Abs(
         Channel_Inputs(CHANNEL_LEFT).Pitch_Rate - Channel_Inputs(CHANNEL_CENTER).Pitch_Rate
      );
      Report.Max_Diff_Yaw := Ada.Numerics.Elementary_Functions.Abs(
         Channel_Inputs(CHANNEL_LEFT).Yaw_Rate - Channel_Inputs(CHANNEL_CENTER).Yaw_Rate
      );
      Report.Max_Diff_Airspeed := Ada.Numerics.Elementary_Functions.Abs(
         Channel_Inputs(CHANNEL_LEFT).Airspeed - Channel_Inputs(CHANNEL_CENTER).Airspeed
      );
      Report.Max_Diff_Altitude := Ada.Numerics.Elementary_Functions.Abs(
         Channel_Inputs(CHANNEL_LEFT).Altitude - Channel_Inputs(CHANNEL_CENTER).Altitude
      );
      return Report;
   end Get_Discrepancy_Report;

   function Is_System_Operational return Boolean is
   begin
      return System_Operational;
   end Is_System_Operational;

   procedure Reset_Manager is
   begin
      Channel_States := (others => HEALTH_FAILED);
      Channel_Inputs := (others => (others => 0.0));
      System_Operational := False;
   end Reset_Manager;

end Redundancy_Manager;
