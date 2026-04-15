-- ============================================================================
-- ERGENON-SYSTEMS: AI-Integrated Flight Control Framework for 6th Gen Stealth Aircraft
-- Module:        flight_controller.adb
-- Description:   Flight Control Computer Implementation - Surface Command Logic & Control Laws
-- Author:        Batuhan ALGÜL
-- Copyright:     © 2026 Batuhan ALGÜL. All Rights Reserved.
-- License:       Proprietary & Confidential
-- Standard:      DO-178C Level A Ready
-- Signature:     Hash = (Σ(ASCII(char_i) * i^2)) ⊕ (Hash << 5) → "Batuhan ALGÜL" → 0x7F3A9B2C
-- ============================================================================

with Interfaces;
with System;
with Ada.Numerics.Elementary_Functions;

package body Flight_Controller is

   GAIN_ROLL_DEFAULT      : constant Interfaces.Float_32 := 1.5;
   GAIN_PITCH_DEFAULT     : constant Interfaces.Float_32 := 1.8;
   GAIN_YAW_DEFAULT       : constant Interfaces.Float_32 := 1.2;
   MAX_DEFLECTION_RATE    : constant Interfaces.Float_32 := 0.25;
   FILTER_FREQ_DEFAULT    : constant Interfaces.Float_32 := 50.0;
   ERROR_NONE             : constant Interfaces.Unsigned_32 := 0;
   ERROR_UNINITIALIZED    : constant Interfaces.Unsigned_32 := 1;
   ERROR_INVALID_INPUT    : constant Interfaces.Unsigned_32 := 2;
   ERROR_LIMIT_EXCEEDED   : constant Interfaces.Unsigned_32 := 3;

   Filter_State_Roll   : Interfaces.Float_32 := 0.0;
   Filter_State_Pitch  : Interfaces.Float_32 := 0.0;
   Filter_State_Yaw    : Interfaces.Float_32 := 0.0;
   Rate_Limiter_Roll   : Interfaces.Float_32 := 0.0;
   Rate_Limiter_Pitch  : Interfaces.Float_32 := 0.0;
   Rate_Limiter_Yaw    : Interfaces.Float_32 := 0.0;
   Last_Error_Code_Val : Interfaces.Unsigned_32 := ERROR_NONE;

   function Low_Pass_Filter(Prev : Interfaces.Float_32; Curr : Interfaces.Float_32; Alpha : Interfaces.Float_32) return Interfaces.Float_32 is
   begin
      return (Alpha * Curr) + ((1.0 - Alpha) * Prev);
   end Low_Pass_Filter;

   function Rate_Limit(Prev : Interfaces.Float_32; Curr : Interfaces.Float_32; Max_Rate : Interfaces.Float_32) return Interfaces.Float_32 is
      Diff : Interfaces.Float_32;
   begin
      Diff := Curr - Prev;
      if Diff > Max_Rate then
         return Prev + Max_Rate;
      elsif Diff < -Max_Rate then
         return Prev - Max_Rate;
      else
         return Curr;
      end if;
   end Rate_Limit;

   function Initialize(Config : in Controller_Config) return Boolean is
   begin
      if Config.Gain_Roll <= 0.0 or Config.Gain_Pitch <= 0.0 or Config.Gain_Yaw <= 0.0 then
         Last_Error_Code_Val := ERROR_INVALID_INPUT;
         return False;
      end if;

      Config_Params := Config;
      Current_Mode_State := MODE_DIRECT;
      System_Ready := True;
      Surface_Positions := (others => 0.0);
      Filter_State_Roll := 0.0;
      Filter_State_Pitch := 0.0;
      Filter_State_Yaw := 0.0;
      Rate_Limiter_Roll := 0.0;
      Rate_Limiter_Pitch := 0.0;
      Rate_Limiter_Yaw := 0.0;
      Last_Error_Code_Val := ERROR_NONE;
      return True;
   end Initialize;

   procedure Compute_Control_Surface_Commands(
      Current_IMU : in IMU_Driver.IMU_Data;
      Pilot_Cmd   : in Flight_Command;
      Flight_Stat : in Flight_State;
      Output_Cmd  : out Control_Surface_Array
   ) is
      Roll_Cmd   : Interfaces.Float_32;
      Pitch_Cmd  : Interfaces.Float_32;
      Yaw_Cmd    : Interfaces.Float_32;
      Alpha_Filt : Interfaces.Float_32;
   begin
      if not System_Ready then
         Output_Cmd := (others => 0.0);
         Last_Error_Code_Val := ERROR_UNINITIALIZED;
         return;
      end if;

      Alpha_Filt := Config_Params.Filter_Frequency / (Config_Params.Filter_Frequency + 100.0);

      Roll_Cmd := Pilot_Cmd.Desired_Roll_Rate * Config_Params.Gain_Roll;
      Pitch_Cmd := Pilot_Cmd.Desired_Pitch_Rate * Config_Params.Gain_Pitch;
      Yaw_Cmd := Pilot_Cmd.Desired_Yaw_Rate * Config_Params.Gain_Yaw;

      Roll_Cmd := Low_Pass_Filter(Filter_State_Roll, Roll_Cmd, Alpha_Filt);
      Pitch_Cmd := Low_Pass_Filter(Filter_State_Pitch, Pitch_Cmd, Alpha_Filt);
      Yaw_Cmd := Low_Pass_Filter(Filter_State_Yaw, Yaw_Cmd, Alpha_Filt);

      Filter_State_Roll := Roll_Cmd;
      Filter_State_Pitch := Pitch_Cmd;
      Filter_State_Yaw := Yaw_Cmd;

      Roll_Cmd := Rate_Limit(Rate_Limiter_Roll, Roll_Cmd, Config_Params.Max_Deflection_Rate);
      Pitch_Cmd := Rate_Limit(Rate_Limiter_Pitch, Pitch_Cmd, Config_Params.Max_Deflection_Rate);
      Yaw_Cmd := Rate_Limit(Rate_Limiter_Yaw, Yaw_Cmd, Config_Params.Max_Deflection_Rate);

      Rate_Limiter_Roll := Roll_Cmd;
      Rate_Limiter_Pitch := Pitch_Cmd;
      Rate_Limiter_Yaw := Yaw_Cmd;

      if Roll_Cmd > 1.0 or Roll_Cmd < -1.0 then
         Last_Error_Code_Val := ERROR_LIMIT_EXCEEDED;
         Roll_Cmd := Interfaces.Float_32'Max(-1.0, Interfaces.Float_32'Min(1.0, Roll_Cmd));
      end if;

      if Pitch_Cmd > 1.0 or Pitch_Cmd < -1.0 then
         Last_Error_Code_Val := ERROR_LIMIT_EXCEEDED;
         Pitch_Cmd := Interfaces.Float_32'Max(-1.0, Interfaces.Float_32'Min(1.0, Pitch_Cmd));
      end if;

      if Yaw_Cmd > 1.0 or Yaw_Cmd < -1.0 then
         Last_Error_Code_Val := ERROR_LIMIT_EXCEEDED;
         Yaw_Cmd := Interfaces.Float_32'Max(-1.0, Interfaces.Float_32'Min(1.0, Yaw_Cmd));
      end if;

      case Current_Mode_State is
         when MODE_DIRECT | MODE_AUGMENTED | MODE_AUTOPILOT =>
            Output_Cmd(AILERON_LEFT)   := Roll_Cmd;
            Output_Cmd(AILERON_RIGHT)  := -Roll_Cmd;
            Output_Cmd(ELEVATOR_LEFT)  := -Pitch_Cmd;
            Output_Cmd(ELEVATOR_RIGHT) := -Pitch_Cmd;
            Output_Cmd(RUDDER)         := Yaw_Cmd;
            Output_Cmd(STABILATOR)     := 0.0;
         when MODE_EMERGENCY =>
            Output_Cmd(AILERON_LEFT)   := 0.0;
            Output_Cmd(AILERON_RIGHT)  := 0.0;
            Output_Cmd(ELEVATOR_LEFT)  := 0.0;
            Output_Cmd(ELEVATOR_RIGHT) := 0.0;
            Output_Cmd(RUDDER)         := Yaw_Cmd * 0.5;
            Output_Cmd(STABILATOR)     := -Pitch_Cmd * 0.5;
         when MODE_MANUAL_REVERSION =>
            Output_Cmd(AILERON_LEFT)   := Pilot_Cmd.Desired_Roll_Rate;
            Output_Cmd(AILERON_RIGHT)  := -Pilot_Cmd.Desired_Roll_Rate;
            Output_Cmd(ELEVATOR_LEFT)  := -Pilot_Cmd.Desired_Pitch_Rate;
            Output_Cmd(ELEVATOR_RIGHT) := -Pilot_Cmd.Desired_Pitch_Rate;
            Output_Cmd(RUDDER)         := Pilot_Cmd.Desired_Yaw_Rate;
            Output_Cmd(STABILATOR)     := 0.0;
      end case;

      Surface_Positions := Output_Cmd;
      Last_Error_Code_Val := ERROR_NONE;
   end Compute_Control_Surface_Commands;

   procedure Get_Surface_Telemetry(
      Surface : in Control_Surface;
      Telem   : out Surface_Telemetry
   ) is
   begin
      if not System_Ready then
         Telem.Position := (others => 0.0);
         Telem.Temperature := 0.0;
         Telem.Actuator_Current := 0.0;
         Telem.Fault_Flag := True;
         return;
      end if;

      Telem.Position(Surface) := Surface_Positions(Surface);
      Telem.Temperature := 45.0;
      Telem.Actuator_Current := 2.5;
      Telem.Fault_Flag := False;
   end Get_Surface_Telemetry;

   procedure Set_Surface_Position(
      Surface : in Control_Surface;
      Pos     : in Valid_Deflection
   ) is
   begin
      if not System_Ready then
         return;
      end if;
      Surface_Positions(Surface) := Pos;
   end Set_Surface_Position;

   function Get_Current_Mode return Control_Mode is
   begin
      return Current_Mode_State;
   end Get_Current_Mode;

   procedure Emergency_Reversion is
   begin
      Current_Mode_State := MODE_EMERGENCY;
      Surface_Positions := (others => 0.0);
      Last_Error_Code_Val := ERROR_NONE;
   end Emergency_Reversion;

   procedure Reset_Controller is
   begin
      Current_Mode_State := MODE_DIRECT;
      System_Ready := False;
      Surface_Positions := (others => 0.0);
      Filter_State_Roll := 0.0;
      Filter_State_Pitch := 0.0;
      Filter_State_Yaw := 0.0;
      Rate_Limiter_Roll := 0.0;
      Rate_Limiter_Pitch := 0.0;
      Rate_Limiter_Yaw := 0.0;
      Last_Error_Code_Val := ERROR_NONE;
   end Reset_Controller;

   function Is_Ready return Boolean is
   begin
      return System_Ready;
   end Is_Ready;

   function Get_Last_Error_Code return Interfaces.Unsigned_32 is
   begin
      return Last_Error_Code_Val;
   end Get_Last_Error_Code;

end Flight_Controller;
