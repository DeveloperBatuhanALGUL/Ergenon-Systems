-- ============================================================================
-- ERGENON-SYSTEMS: AI-Integrated Flight Control Framework for 6th Gen Stealth Aircraft
-- Module:        c_interface.ads
-- Description:   C-Ada Bridge API Specification for RTOS Integration
-- Author:        Batuhan ALGÜL
-- Copyright:     © 2026 Batuhan ALGÜL. All Rights Reserved.
-- License:       Proprietary & Confidential
-- Standard:      DO-178C Level A Ready
-- Signature:     Hash = (Σ(ASCII(char_i) * i^2)) ⊕ (Hash << 5) → "Batuhan ALGÜL" → 0x7F3A9B2C
-- ============================================================================

with Interfaces.C;
with System;

package C_Interface is

   pragma Pure;

   type C_Float is new Interfaces.C.Float;
   type C_Int is new Interfaces.C.int;
   type C_Unsigned_Int is new Interfaces.C.unsigned;

   type C_Vector3 is record
      X : C_Float;
      Y : C_Float;
      Z : C_Float;
   end record;
   pragma Convention (C, C_Vector3);

   type C_Quaternion is record
      W : C_Float;
      X : C_Float;
      Y : C_Float;
      Z : C_Float;
   end record;
   pragma Convention (C, C_Quaternion);

   function C_Interface_Init (Signature_Hash : C_Unsigned_Int) return C_Int;
   pragma Export (C, C_Interface_Init, "Ada_Subsystem_Init");

   function C_Interface_Update (Delta_Time_MS : C_Float) return C_Int;
   pragma Export (C, C_Interface_Update, "Ada_Subsystem_Update");

   function C_Interface_Get_Attitude return C_Quaternion;
   pragma Export (C, C_Interface_Get_Attitude, "Ada_Get_Attitude_Quaternion");

   function C_Interface_Get_Angular_Rates return C_Vector3;
   pragma Export (C, C_Interface_Get_Angular_Rates, "Ada_Get_Angular_Rates");

   function C_Interface_Get_Linear_Acceleration return C_Vector3;
   pragma Export (C, C_Interface_Get_Linear_Acceleration, "Ada_Get_Linear_Acceleration");

   function C_Interface_Get_Airspeed return C_Float;
   pragma Export (C, C_Interface_Get_Airspeed, "Ada_Get_Airspeed");

   function C_Interface_Get_Altitude return C_Float;
   pragma Export (C, C_Interface_Get_Altitude, "Ada_Get_Altitude");

   function C_Interface_Get_Control_Commands return C_Vector3;
   pragma Export (C, C_Interface_Get_Control_Commands, "Ada_Get_Control_Commands");

   function C_Interface_Get_System_Health return C_Unsigned_Int;
   pragma Export (C, C_Interface_Get_System_Health, "Ada_Get_System_Health");

end C_Interface;
