within BobLib.Resources.VehicleDefn;

record DWBCStabar_DWBCStabarRecord

  import BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.MassRecord;

  import Aero = BobLib.Resources.VehicleRecord.Aero;
  import TireModel = BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.Tire.MF52;
  import Wheel = BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.Tire;
  import Rack = BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.SteeringRack;
  import Stabar = BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.Stabar;
  import DW = BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.DoubleWishbone;
  import Axle = BobLib.Resources.VehicleRecord.Chassis.Suspension;

  parameter Axle.AxleDW_BC_StabarRecord pFrAxleDW(
      bellcrankPivot = {-0.042144464098, 0.250754351932, 0.370010000136},
      bellcrankPivotAxis = {0.95754655, -0.26587315, -0.11142744},
      bellcrankRodPickup = {-0.014493326106, 0.348410770284, 0.374614186762},
      bellcrankShockPickup = {-0.01102742905, 0.34553503283, 0.411259910778},
      bellcrankStabarPickup = {-0.029010173628, 0.2971411507, 0.37219698865},
      rodPickup = 2,
      shockPickup = 3,
      stabarPickup = 1,
      shockMount = {-0.020673469702, 0.247847085458, 0.561456926868},
      springTable = [0, 0; 1, 26269.02525],
      springFreeLength = 0.2,
      damperTable = [-1, -850.0; 0, 0; 1, 850]);

  parameter Stabar.StabarRecord pFrStabar(
      leftArmEnd = {-0.03682914, 0.2667, 0.11597939},
      leftBarEnd = {-0.10664664, 0.2667, 0.11811},
      barRate = 1);

  parameter Wheel.Templates.PartialWheelRecord pFrPartialWheel(
      R0 = 0.2045,
      rimR0 = 0.2045*0.625,
      rimWidth = 0.2045*0.625*1.4,
      staticAlpha = 0,
      staticGamma = 0);

  parameter Rack.RackAndPinionRecord pFrRack(
      leftPickup = {0.05715, 0.2260092, 0.1137158},
      cFactor = 0.0889);

  parameter DW.WishboneUprightLoopRecord pFrDW(
      upperFore_i = {0.1016, 0.237744, 0.2143252},
      upperAft_i = {-0.0680974, 0.2356358, 0.215138},
      lowerFore_i = {0.1016, 0.226314, 0.08001},
      lowerAft_i = {-0.0762, 0.226314, 0.08001},
      upper_o = {-0.0092964, 0.5420106, 0.2679954},
      lower_o = {0.0029972, 0.562991, 0.1139952},
      tie_o = {0.0569976, 0.546989, 0.1522222},
      wheelCenter = {0, 0.606110767456, 0.199898},
      rodToLower = true,
      rodMount = {0.006762552642, 0.525610676234, 0.134465050856});

  parameter Axle.Templates.AxleMassRecord pFrAxleMass(
      unsprungMass = MassRecord(m = 7.8160579, rCM = {-0.0061298, 0.60174377, 0.19797979}, inertia = {{0.10580066, 0.00038293, 0.00058877}, {0.00038293, 0.16064008, -0.00075416}, {0.00058877, -0.00075416, 0.10801766}}),
      ucaMass = MassRecord(m = 0.55776965, rCM = {-0.00187916, 0.45854113, 0.25343195}, inertia = {{0.00700626, -0.00058434, -0.0001073}, {-0.00058434, 0.0011635, 0.00116272}, {-0.0001073, 0.00116272, 0.00762498}}),
      lcaMass = MassRecord(m = 0.5182905, rCM = {0.00803455, 0.40581792, 0.09882118}, inertia = {{0.00649759, -0.00017572, -1.824e-05}, {-0.00017572, 0.00145437, 0.00068888}, {-1.824e-05, 0.00068888, 0.00776251}}),
      tieMass = MassRecord(m = 0.13459415, rCM = {0.05709287, 0.34616483, 0.1281302}, inertia = {{0.00178949, -8.3e-07, -1e-07}, {-8.3e-07, 2.994e-05, 0.00021109}, {-1e-07, 0.000211, 0.001764}}));

  parameter Wheel.Wheel1DOF_YRecord pFrTire1DOF_YParams(
      wheelJ = 0.02);

  parameter Wheel.Wheel1DOF_ZRecord pFrTire1DOF_ZParams(
      wheelC = 98947,
      wheelD = 115.844);

  parameter TireModel.MF52Record pFrTireModel(
      fxCombined = TireModel.CombinedSlip.FxCombinedRecord(RBX1 = 8.151136, RBX2 = 5.388063, RCX1 = 1.122399, REX1 = 0.052014, REX2 = -0.89845, RHX1 = 0.0),
      fxPure = TireModel.PureSlip.FxPureRecord(LFZO = 1.0, LGAX = 1.0, PCX1 = 1.53041, PDX1 = 2.597991, PDX2 = -0.618826, PDX3 = 11.156379, PKX1 = 55.079922, PKX2 = -1.7e-05, PKX3 = -0.16185, PHX1 = 0.0, PHX2 = 0.0, PVX1 = 0.0, PVX2 = 0.0, PEX1 = 0.0, PEX2 = 0.141806, PEX3 = -1.93495, PEX4 = 0.044722, LCX = 1.0, LMUX = 1.0, LKX = 1.0, LHX = 1.0, LVX = 1.0, LEX = 1.0, LXAL = 1.0),
      fyCombined = TireModel.CombinedSlip.FyCombinedRecord(RBY1 = 14.628, RBY2 = 10.4, RBY3 = 0.0, RCY1 = 1.044, REY1 = 0.048, REY2 = 0.025, RHY1 = 0.0, RHY2 = 0.0, RVY1 = 0.0, RVY2 = 0.0, RVY3 = 0.0, RVY4 = 0.0, RVY5 = 0.0, RVY6 = 0.0),
      fyPure = TireModel.PureSlip.FyPureRecord(LFZO = 1.0, LGAY = 1.0, PCY1 = 1.53041, PDY1 = -2.40275, PDY2 = 0.343535, PDY3 = 3.89743, PKY1 = -53.2421, PKY2 = 2.38205, PKY3 = 1.36502, PHY1 = 0.0, PHY2 = 0.0, PHY3 = 0.0, PVY1 = 0.0, PVY2 = 0.0, PVY3 = 0.0, PVY4 = 0.0, PEY1 = 0.0, PEY2 = -0.280762, PEY3 = 0.70403, PEY4 = -0.478297, LCY = 1.0, LMUY = 1.0, LEY = 1.0, LKY = 1.0, LHY = 1.0, LVY = 1.0, LYKA = 1.0, LVYKA = 1.0),
      mxCombined = TireModel.CombinedSlip.MxCombinedRecord(),
      mxPure = TireModel.PureSlip.MxPureRecord(QSX1 = -0.0130807, QSX2 = 0.0, QSX3 = 0.0587803, LMX = 1.0, LVMX = 1.0),
      myCombined = TireModel.CombinedSlip.MyCombinedRecord(),
      myPure = TireModel.PureSlip.MyPureRecord(QSY1 = 0.0, QSY2 = 0.0, QSY3 = 0.0, QSY4 = 0.0, Vref = 12.0, LMY = 1.0),
      mzCombined = TireModel.CombinedSlip.MzCombinedRecord(SSZ1 = 0.0, SSZ2 = 0.0, SSZ3 = 0.0, SSZ4 = 0.0, RVY1 = 0.0, RVY2 = 0.0, RVY3 = 0.0, RVY4 = 0.0, RVY5 = 0.0, RVY6 = 0.0, LS = 1.0, LVYKA = 1.0),
      mzPure = TireModel.PureSlip.MzPureRecord(QBZ1 = 8.22843, QBZ2 = 2.98676, QBZ3 = -3.57739, QBZ4 = -0.429117, QBZ5 = 0.433125, QCZ1 = 1.41359, QDZ1 = 0.152526, QDZ2 = -0.0381101, QDZ3 = 0.387762, QDZ4 = -3.95699, QEZ1 = -0.239731, QEZ2 = 1.29253, QEZ3 = -1.21298, QEZ4 = 0.0, QEZ5 = 0.0, QHZ1 = 0.0, QHZ2 = 0.0, QHZ3 = 0.0, QHZ4 = 0.0, QBZ9 = 0.0, QBZ10 = -1.72926, QDZ6 = 0.00604966, QDZ7 = -0.000116241, QDZ8 = -2.33359, QDZ9 = -0.0379755, LTR = 1.0, LRES = 1.0, LKY = 1.0, LMUY = 1.0, LGAZ = 1.0),
      setup = TireModel.SetupRecord(FNOMIN = 650.0, UNLOADED_RADIUS = pFrPartialWheel.R0));

  parameter Axle.AxleDW_BC_StabarRecord pRrAxleDW(
      bellcrankPivot = {-1.39886851, 0.29230126, 0.1016},
      bellcrankPivotAxis = {0.887962410598, 0.302708516859, 0.346251803151},
      bellcrankRodPickup = {-1.41267566, 0.35197317, 0.08484064},
      bellcrankShockPickup = {-1.43801295, 0.36137285, 0.14160048},
      bellcrankStabarPickup = {-1.41346984, 0.31057564, 0.12306883},
      rodPickup = 1,
      shockPickup = 2,
      stabarPickup = 3,
      shockMount = {-1.50192144, 0.28884688, 0.36889916},
      springTable = [0, 0; 1, 43781.70875],
      springFreeLength = 0.265,
      damperTable = [-1, -1300.0; 0, 0; 1, 1300]);

  parameter Stabar.StabarRecord pRrStabar(
      leftArmEnd = {-1.43001283, 0.3032125, 0.4054766},
      leftBarEnd = {-1.3925183, 0.3032125, 0.41224196},
      barRate = 1);

  parameter Wheel.Templates.PartialWheelRecord pRrPartialWheel(
      R0 = 0.2045,
      rimR0 = 0.2045*0.625,
      rimWidth = 0.2045*0.625*1.4,
      staticAlpha = 0,
      staticGamma = 0);

  parameter Rack.RackAndPinionRecord pRrRack(
      leftPickup = {-1.3763498, 0.2897124, 0.1700022},
      cFactor = 0.0889);

  parameter DW.WishboneUprightLoopRecord pRrDW(
      upperFore_i = {-1.279144, 0.2972308, 0.2482342},
      upperAft_i = {-1.4993874, 0.283845, 0.2434336},
      lowerFore_i = {-1.3142214, 0.283464, 0.086868},
      lowerAft_i = {-1.4998192, 0.2835148, 0.0872236},
      upper_o = {-1.5540736, 0.5267706, 0.29464},
      lower_o = {-1.55448, 0.57658, 0.116078},
      tie_o = {-1.45796, 0.5823966, 0.2143506},
      wheelCenter = {-1.5494, 0.60611077, 0.199898},
      rodToLower = false,
      rodMount = {-1.53509479, 0.50330883, 0.26648017});

  parameter Axle.Templates.AxleMassRecord pRrAxleMass(
      unsprungMass = MassRecord(m = 7.35802418, rCM = {-1.54948701, 0.60559861, 0.20104023}, inertia = {{0.10429603, 8.706e-05, 4.516e-05}, {8.706e-05, 0.15655532, -0.00147057}, {4.516e-05, -0.00147057, 0.10327132}}),
      ucaMass = MassRecord(m = 0.34927896, rCM = {-1.47556121, 0.42263392, 0.27196898}, inertia = {{0.00248293, -0.00169357, -0.00031129}, {-0.00169357, 0.00276385, 0.00045612}, {-0.00031129, 0.00045612, 0.00504556}}),
      lcaMass = MassRecord(m = 0.46311564, rCM = {-1.47625848, 0.4301135, 0.10156115}, inertia = {{0.00434917, -0.00226155, -0.0002268}, {-0.00226155, 0.00268231, 0.00042397}, {-0.0002268, 0.00042397, 0.0069183}}),
      tieMass = MassRecord(m = 0.13293714, rCM = {-1.40704567, 0.39979876, 0.18668273}, inertia = {{0.00146339, -0.00039766, -6.025e-05}, {-0.00039766, 0.00014812, 0.0002161}, {-6.025e-05, 0.0002161, 0.00154153}}));

  parameter Wheel.Wheel1DOF_YRecord pRrTire1DOF_YParams(
      wheelJ = 0.02);

  parameter Wheel.Wheel1DOF_ZRecord pRrTire1DOF_ZParams(
      wheelC = 98947,
      wheelD = 115.844);

  parameter TireModel.MF52Record pRrTireModel(
      fxCombined = TireModel.CombinedSlip.FxCombinedRecord(RBX1 = 8.151136, RBX2 = 5.388063, RCX1 = 1.122399, REX1 = 0.052014, REX2 = -0.89845, RHX1 = 0.0),
      fxPure = TireModel.PureSlip.FxPureRecord(LFZO = 1.0, LGAX = 1.0, PCX1 = 1.53041, PDX1 = 2.597991, PDX2 = -0.618826, PDX3 = 11.156379, PKX1 = 55.079922, PKX2 = -1.7e-05, PKX3 = -0.16185, PHX1 = 0.0, PHX2 = 0.0, PVX1 = 0.0, PVX2 = 0.0, PEX1 = 0.0, PEX2 = 0.141806, PEX3 = -1.93495, PEX4 = 0.044722, LCX = 1.0, LMUX = 1.0, LKX = 1.0, LHX = 1.0, LVX = 1.0, LEX = 1.0, LXAL = 1.0),
      fyCombined = TireModel.CombinedSlip.FyCombinedRecord(RBY1 = 14.628, RBY2 = 10.4, RBY3 = 0.0, RCY1 = 1.044, REY1 = 0.048, REY2 = 0.025, RHY1 = 0.0, RHY2 = 0.0, RVY1 = 0.0, RVY2 = 0.0, RVY3 = 0.0, RVY4 = 0.0, RVY5 = 0.0, RVY6 = 0.0),
      fyPure = TireModel.PureSlip.FyPureRecord(LFZO = 1.0, LGAY = 1.0, PCY1 = 1.53041, PDY1 = -2.40275, PDY2 = 0.343535, PDY3 = 3.89743, PKY1 = -53.2421, PKY2 = 2.38205, PKY3 = 1.36502, PHY1 = 0.0, PHY2 = 0.0, PHY3 = 0.0, PVY1 = 0.0, PVY2 = 0.0, PVY3 = 0.0, PVY4 = 0.0, PEY1 = 0.0, PEY2 = -0.280762, PEY3 = 0.70403, PEY4 = -0.478297, LCY = 1.0, LMUY = 1.0, LEY = 1.0, LKY = 1.0, LHY = 1.0, LVY = 1.0, LYKA = 1.0, LVYKA = 1.0),
      mxCombined = TireModel.CombinedSlip.MxCombinedRecord(),
      mxPure = TireModel.PureSlip.MxPureRecord(QSX1 = -0.0130807, QSX2 = 0.0, QSX3 = 0.0587803, LMX = 1.0, LVMX = 1.0),
      myCombined = TireModel.CombinedSlip.MyCombinedRecord(),
      myPure = TireModel.PureSlip.MyPureRecord(QSY1 = 0.0, QSY2 = 0.0, QSY3 = 0.0, QSY4 = 0.0, Vref = 12.0, LMY = 1.0),
      mzCombined = TireModel.CombinedSlip.MzCombinedRecord(SSZ1 = 0.0, SSZ2 = 0.0, SSZ3 = 0.0, SSZ4 = 0.0, RVY1 = 0.0, RVY2 = 0.0, RVY3 = 0.0, RVY4 = 0.0, RVY5 = 0.0, RVY6 = 0.0, LS = 1.0, LVYKA = 1.0),
      mzPure = TireModel.PureSlip.MzPureRecord(QBZ1 = 8.22843, QBZ2 = 2.98676, QBZ3 = -3.57739, QBZ4 = -0.429117, QBZ5 = 0.433125, QCZ1 = 1.41359, QDZ1 = 0.152526, QDZ2 = -0.0381101, QDZ3 = 0.387762, QDZ4 = -3.95699, QEZ1 = -0.239731, QEZ2 = 1.29253, QEZ3 = -1.21298, QEZ4 = 0.0, QEZ5 = 0.0, QHZ1 = 0.0, QHZ2 = 0.0, QHZ3 = 0.0, QHZ4 = 0.0, QBZ9 = 0.0, QBZ10 = -1.72926, QDZ6 = 0.00604966, QDZ7 = -0.000116241, QDZ8 = -2.33359, QDZ9 = -0.0379755, LTR = 1.0, LRES = 1.0, LKY = 1.0, LMUY = 1.0, LGAZ = 1.0),
      setup = TireModel.SetupRecord(FNOMIN = 650.0, UNLOADED_RADIUS = pRrPartialWheel.R0));

  parameter MassRecord pSprungMass(
      m = 200,
      rCM = {-0.7747, 0, 0.2794},
      inertia = {{30, 1, 1}, {1, 40, 1}, {1, 1, 50}});

  parameter Aero.CFDAeroMapRecord pAero(
      referenceSpeed = 15.0,
      frontRideHeightGrid = {0.03556, 0.05334, 0.07112, 0.0889, 0.10668},
      rearRideHeightGrid = {0.04191, 0.0635, 0.08382, 0.10414, 0.12573},
      dragTable = {{80.7986, 80.9749, 81.8435, 79.6167, 77.250725}, {82.2735, 82.36745, 86.1779, 86.0919, 85.1031}, {79.6194, 83.76, 86.2301, 88.2285, 90.0656}, {78.1912, 81.1352, 84.552, 87.3099, 89.4312}, {77.8347, 78.5104, 82.8739, 86.3913, 87.4556}},
      downforceTable = {{161.7379, 153.1401, 132.6786, 107.9693, 81.71566875}, {147.2059, 150.60985, 170.7603, 165.86, 150.6537}, {131.9303, 148.0796, 159.8995, 169.7841, 175.6373}, {119.2166, 130.7024, 143.2776, 152.8575, 166.1884}, {109.7918, 113.3252, 126.6557, 135.9309, 152.0952}},
      mxTable = {{67.3388, 64.4234, 59.0409, 52.4527, 45.4527375}, {62.1824, 63.3701, 70.297, 70.1172, 64.5983}, {56.6292, 62.3168, 66.4454, 70.5332, 72.301}, {52.7975, 57.1988, 61.9635, 65.7843, 70.1818}, {49.1021, 52.0808, 57.4816, 61.0354, 65.61}},
      myTable = {{-251.2587, -259.5945, -271.8122, -274.5288, -277.4151875}, {-230.6561, -250.1893, -268.0828, -271.5582, -278.5465}, {-216.6613, -240.7841, -256.2496, -268.5392, -276.6542}, {-202.8485, -217.5691, -235.5897, -246.9616, -264.2709}, {-193.9678, -194.3541, -214.9298, -225.384, -249.1606}},
      mzTable = {{30.5855, 35.2995, 36.4593, 39.6766, 43.09498125}, {20.2251, 27.17715, 26.6943, 38.6983, 38.7101}, {19.6735, 19.0548, 20.2878, 24.1933, 25.0432}, {19.6157, 21.212, 21.7999, 24.0418, 22.7023}, {19.0574, 23.3692, 23.312, 23.8903, 20.9816}});

end DWBCStabar_DWBCStabarRecord;
