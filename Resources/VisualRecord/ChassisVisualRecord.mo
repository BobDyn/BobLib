within BobLib.Resources.VisualRecord;

record ChassisVisualRecord

  // ============================================================
  // FRONT AXLE
  // ============================================================

  // Wishbone (inner)
  Real frUpperFore_i[3];
  Real frUpperAft_i[3];
  Real frLowerFore_i[3];
  Real frLowerAft_i[3];

  // Upright (outer)
  Real frUpper_o[3];
  Real frLower_o[3];

  // Steering
  Real frTie_i[3];
  Real frTie_o[3];

  // Wheel
  Real frWheelCenter[3];
  Real frTire_ex[3];
  Real frTire_ey[3];

//  // Bellcrank
//  Real frBellcrankPivot[3];
//  Real frBellcrankRod_i[3];
//  Real frBellcrankShock_i[3];
//  Real frBellcrankLink1_i[3];
//  Real frBellcrankLink2_i[3];
//  Real frBellcrankLink3_i[3];

//  // Stabar
//  Real frStabarArm_i[3];
//  Real frStabarBar_i[3];

//  // Rack
//  Real frRackLeft_i[3];
//  Real frRackRight_i[3];


//  // ============================================================
//  // REAR AXLE
//  // ============================================================

//  // Wishbone (inner)
//  Real rrUpperFore_i[3];
//  Real rrUpperAft_i[3];
//  Real rrLowerFore_i[3];
//  Real rrLowerAft_i[3];

//  // Upright (outer)
//  Real rrUpper_o[3];
//  Real rrLower_o[3];

//  // Toe link / rear steering (if applicable)
//  Real rrTie_o[3];

//  // Wheel
//  Real rrWheelCenter[3];

//  // Bellcrank
//  Real rrBellcrankPivot[3];
//  Real rrBellcrankRod_i[3];
//  Real rrBellcrankShock_i[3];
//  Real rrBellcrankLink1_i[3];
//  Real rrBellcrankLink2_i[3];
//  Real rrBellcrankLink3_i[3];

//  // Stabar
//  Real rrStabarArm_i[3];
//  Real rrStabarBar_i[3];


//  // ============================================================
//  // GLOBAL (OPTIONAL BUT VERY USEFUL)
//  // ============================================================

//  // Center of gravity
//  Real cg[3];

//  // Optional chassis reference (helps camera / alignment)
//  Real chassisOrigin[3];

end ChassisVisualRecord;
