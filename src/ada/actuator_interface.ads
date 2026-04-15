-- ============================================================================
-- ERGENON-SYSTEMS: AI-Integrated Flight Control Framework for 6th Gen Stealth Aircraft
-- Module:        actuator_interface.ads
-- Description:   Actuator Control & PWM Signal Generation Specification
-- Author:        Batuhan ALGÜL
-- Copyright:     © 2026 Batuhan ALGÜL. All Rights Reserved.
-- License:       Proprietary & Confidential
-- Standard:      DO-178C Level A Ready
-- Signature:     Hash = (Σ(ASCII(char_i) * i^2)) ⊕ (Hash << 5) → "Batuhan ALGÜL" → 0x7F3A9B2C
-- ============================================================================

with Interfaces;
with System;
with Flight_Controller;

package Actuator_Interface is

   pragma Preelaborate(Actuator_Interface);

   type Actuator_ID is (
      ACTUATOR_AILERON_LEFT,
      ACTUATOR_AILERON_RIGHT,
      ACTUATOR_ELEVATOR_LEFT,
      ACTUATOR_ELEVATOR_RIGHT,
      ACTUATOR_RUDDER,
      ACTUATOR_THROTTLE_LEFT,
      ACTUATOR_THROTTLE_RIGHT
   );

   type Actuator_Channel is range 1 .. 7;

   type PWM_Range is record
      Min_Pulse_us      : Interfaces.Unsigned_16;
      Max_Pulse_us      : Interfaces.Unsigned_16;
      Center_Pulse_us   : Interfaces.Unsigned_16;
      Update_Rate_Hz    : Interfaces.Unsigned_16;
   end record;

   type Actuator_State is record
      Current_Position_Percentage : Interfaces.Float_32;
      Target_Position_Percentage  : Interfaces.Float_32;
      Current_Draw_mA             : Interfaces.Float_32;
      Temperature_Celsius         : Interfaces.Float_32;
      Is_Feedback_Valid           : Boolean;
   end record;

   type Actuator_State_Map is array (Actuator_ID) of Actuator_State;

   type Actuator_Config is record
      PWM_Ranges              : PWM_Range;
      Max_Rate_of_Change_Pct  : Interfaces.Float_32;
      Deadzone_Threshold      : Interfaces.Float_32;
      Emergency_Centering     : Boolean;
   end record;

   function Initialize(Config : in Actuator_Config) return Boolean;

   procedure Set_Actuator_Position(
      ID       : in Actuator_ID;
      Position : in Interfaces.Float_32
   );

   function Get_Actuator_State(ID : in Actuator_ID) return Actuator_State;

   procedure Update_Actuators(
      Flight_Cmds : in Flight_Controller.Control_Surface_Array;
      DT_Seconds  : in Interfaces.Float_32
   );

   procedure Emergency_Land(All_Surfaces : in Interfaces.Float_32);

   function Get_PWM_Output_Value(ID : in Actuator_ID) return Interfaces.Unsigned_16;

   function Is_Actuator_Healthy(ID : in Actuator_ID) return Boolean;

   procedure Run_Self_Test;

   function Get_Last_Error return Interfaces.Unsigned_32;

private

   Actuator_States  : Actuator_State_Map;
   System_Ready     : Boolean := False;
   Current_PWM_Values : array (Actuator_ID) of Interfaces.Unsigned_16 := (others => 1500);
   Last_Error_Code  : Interfaces.Unsigned_32 := 0;
   Config_Params    : Actuator_Config;

end Actuator_Interface;
