-- ============================================================================
-- ERGENON-SYSTEMS: AI-Integrated Flight Control Framework for 6th Gen Stealth Aircraft
-- Module:        redundancy_manager.ads
-- Description:   Triple-Modular Redundancy (TMR) and Voting Logic Specification
-- Author:        Batuhan ALGÜL
-- Copyright:     © 2026 Batuhan ALGÜL. All Rights Reserved.
-- License:       Proprietary & Confidential
-- Standard:      DO-178C Level A Ready
-- Signature:     Hash = (Σ(ASCII(char_i) * i^2)) ⊕ (Hash << 5) → "Batuhan ALGÜL" → 0x7F3A9B2C
-- ============================================================================

with Interfaces;
with System;

package Redundancy_Manager is

   pragma Preelaborate(Redundancy_Manager);

   type Channel_ID is (
      CHANNEL_LEFT,
      CHANNEL_CENTER,
      CHANNEL_RIGHT
   );

   type Channel_Health is (
      HEALTH_OK,
      HEALTH_DEGRADED,
      HEALTH_FAILED,
      HEALTH_DISAGREE
   );

   type Channel_Health_Map is array (Channel_ID) of Channel_Health;

   type Voting_Mode is (
      MODE_MID_VALUE_SELECT,
      MODE_AVERAGE,
      MODE_MAJORITY_VOTE
   );

   type Signal_Vector_3 is array (1 .. 3) of Interfaces.Float_32;

   type Channel_Signal_Data is record
      Roll_Rate          : Interfaces.Float_32;
      Pitch_Rate         : Interfaces.Float_32;
      Yaw_Rate           : Interfaces.Float_32;
      Vertical_Accel     : Interfaces.Float_32;
      Lateral_Accel      : Interfaces.Float_32;
      Longitudinal_Accel : Interfaces.Float_32;
      Airspeed           : Interfaces.Float_32;
      Altitude           : Interfaces.Float_32;
      Angle_Of_Attack    : Interfaces.Float_32;
      Angle_Of_Sideslip  : Interfaces.Float_32;
      Mach_Number        : Interfaces.Float_32;
   end record;

   type Channel_Data_Map is array (Channel_ID) of Channel_Signal_Data;

   type Voted_Output is record
      Roll_Rate          : Interfaces.Float_32;
      Pitch_Rate         : Interfaces.Float_32;
      Yaw_Rate           : Interfaces.Float_32;
      Vertical_Accel     : Interfaces.Float_32;
      Lateral_Accel      : Interfaces.Float_32;
      Longitudinal_Accel : Interfaces.Float_32;
      Airspeed           : Interfaces.Float_32;
      Altitude           : Interfaces.Float_32;
      Valid              : Boolean;
      Active_Channels    : Interfaces.Unsigned_8;
   end record;

   type Discrepancy_Info is record
      Max_Diff_Roll      : Interfaces.Float_32;
      Max_Diff_Pitch     : Interfaces.Float_32;
      Max_Diff_Yaw       : Interfaces.Float_32;
      Max_Diff_Airspeed  : Interfaces.Float_32;
      Max_Diff_Altitude  : Interfaces.Float_32;
   end record;

   function Initialize(Voting_Method : in Voting_Mode) return Boolean;

   procedure Update_Channel_Health(
      ID     : in Channel_ID;
      Status : in Channel_Health
   );

   procedure Input_Channel_Data(
      ID   : in Channel_ID;
      Data : in Channel_Signal_Data
   );

   function Compute_Voted_Signals return Voted_Output;

   function Get_Channel_Health(ID : in Channel_ID) return Channel_Health;

   function Get_Discrepancy_Report return Discrepancy_Info;

   function Is_System_Operational return Boolean;

   procedure Reset_Manager;

private

   Current_Voting_Mode : Voting_Mode := MODE_MID_VALUE_SELECT;
   System_Operational  : Boolean := False;
   Channel_States      : Channel_Health_Map := (others => HEALTH_FAILED);
   Channel_Inputs      : Channel_Data_Map;
   Threshold_Roll      : constant Interfaces.Float_32 := 0.05;
   Threshold_Pitch     : constant Interfaces.Float_32 := 0.05;
   Threshold_Yaw       : constant Interfaces.Float_32 := 0.05;
   Threshold_Airspeed  : constant Interfaces.Float_32 := 2.0;
   Threshold_Altitude  : constant Interfaces.Float_32 := 5.0;

end Redundancy_Manager;
