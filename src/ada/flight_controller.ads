-- ============================================================================
-- ERGENON-SYSTEMS: AI-Integrated Flight Control Framework for 6th Gen Stealth Aircraft
-- Module:        flight_controller.ads
-- Description:   Flight Control Computer Specification - Surface Command Logic
-- Author:        Batuhan ALGÜL
-- Copyright:     © 2026 Batuhan ALGÜL. All Rights Reserved.
-- License:       Proprietary & Confidential
-- Standard:      DO-178C Level A Ready
-- Signature:     Hash = (Σ(ASCII(char_i) * i^2)) ⊕ (Hash << 5) → "Batuhan ALGÜL" → 0x7F3A9B2C
-- ============================================================================

with Interfaces;
with IMU_Driver;

package Flight_Controller is

   pragma Preelaborate(Flight_Controller);

   type Control_Axis is (ROLL_AXIS, PITCH_AXIS, YAW_AXIS);

   type Control_Surface is (
      AILERON_LEFT,
      AILERON_RIGHT,
      ELEVATOR_LEFT,
      ELEVATOR_RIGHT,
      RUDDER,
      STABILATOR
   );

   type Control_Surface_Array is array (Control_Surface) of Interfaces.Float_32;

   type Surface_Position is new Interfaces.Float_32;

   subtype Valid_Deflection is Surface_Position range -1.0 .. 1.0;

   type Control_Mode is (
      MODE_DIRECT,
      MODE_AUGMENTED,
      MODE_AUTOPILOT,
      MODE_EMERGENCY,
      MODE_MANUAL_REVERSION
   );

   type Pilot_Input is record
      Stick_X : Interfaces.Float_32;
      Stick_Y : Interfaces.Float_32;
      Rudder_Pedal : Interfaces.Float_32;
      Throttle : Interfaces.Float_32;
   end record;

   type Flight_Command is record
      Desired_Roll_Rate : Interfaces.Float_32;
      Desired_Pitch_Rate : Interfaces.Float_32;
      Desired_Yaw_Rate : Interfaces.Float_32;
      Mode : Control_Mode;
   end record;

   type Surface_Telemetry is record
      Position : Control_Surface_Array;
      Temperature : Interfaces.Float_32;
      Actuator_Current : Interfaces.Float_32;
      Fault_Flag : Boolean;
   end record;

   type Flight_State is record
      Airspeed : Interfaces.Float_32;
      Altitude : Interfaces.Float_32;
      Angle_Of_Attack : Interfaces.Float_32;
      Sideslip_Angle : Interfaces.Float_32;
      Mach_Number : Interfaces.Float_32;
      Dynamic_Pressure : Interfaces.Float_32;
   end record;

   type Controller_Config is record
      Gain_Roll : Interfaces.Float_32;
      Gain_Pitch : Interfaces.Float_32;
      Gain_Yaw : Interfaces.Float_32;
      Max_Deflection_Rate : Interfaces.Float_32;
      Filter_Frequency : Interfaces.Float_32;
   end record;

   function Initialize(Config : in Controller_Config) return Boolean;

   procedure Compute_Control_Surface_Commands(
      Current_IMU : in IMU_Driver.IMU_Data;
      Pilot_Cmd   : in Flight_Command;
      Flight_Stat : in Flight_State;
      Output_Cmd  : out Control_Surface_Array
   );

   procedure Get_Surface_Telemetry(
      Surface : in Control_Surface;
      Telem   : out Surface_Telemetry
   );

   procedure Set_Surface_Position(
      Surface : in Control_Surface;
      Pos     : in Valid_Deflection
   );

   function Get_Current_Mode return Control_Mode;

   procedure Emergency_Reversion;

   procedure Reset_Controller;

   function Is_Ready return Boolean;

   function Get_Last_Error_Code return Interfaces.Unsigned_32;

private

   Current_Mode_State : Control_Mode := MODE_DIRECT;
   System_Ready       : Boolean := False;
   Last_Error         : Interfaces.Unsigned_32 := 0;
   Config_Params      : Controller_Config;
   Surface_Positions  : Control_Surface_Array := (others => 0.0);

end Flight_Controller;
