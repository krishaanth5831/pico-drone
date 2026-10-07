FeatureScript 3083;
import(path : "onshape/std/geometry.fs", version : "3083.0");

/*
 * Pico Drone Frame - parametric quad frame for krishaanth5831/pico-drone.
 *
 * Coordinates (flight orientation): +X = nose, +Y = left, +Z = up.
 * Z = 0 is the TOP face of the deck - the plane every board sits on.
 * Everything structural (deck, battery-bay trusses, arm webs, motor pods)
 * hangs BELOW z = 0, so the frame prints upside-down: deck face on the bed,
 * no supports. The GPS/compass mast is a separate, bolt-on part.
 *
 * Boards keep their pins-down headers: each header's plastic strip drops
 * through a slot in the deck, so the PCB lies flat on the deck and the pins
 * hang into a wiring bay between the deck and the battery.
 *
 * Weight: every outline is cut back to what carries load or supports a board -
 * windows under the boards, Warren-truss battery-bay walls, tapered T-section
 * arms, gussets instead of full rims. Every corner is drawn rounded and every
 * remaining convex edge gets a fillet, so there are no sharp edges.
 *
 * Pico 2 W numbers are from the official datasheet (51 x 21 x 1 mm, 2.1 mm
 * holes on 47 x 11.4, header rows 17.78 apart). Clone-board sizes are typical
 * published values - caliper yours and edit the dialog.
 */

const MM = millimeter;

// ---------------------------------------------------------------- bounds
const B_MOTOR_D    = { (millimeter) : [3, 7.0, 12] } as LengthBoundSpec;
const B_MOTOR_FIT  = { (millimeter) : [-0.5, 0.1, 0.8] } as LengthBoundSpec;
const B_MOTOR_LEN  = { (millimeter) : [8, 20, 35] } as LengthBoundSpec;
const B_MOTOR_RISE = { (millimeter) : [2, 11, 25] } as LengthBoundSpec;
const B_PROP_D     = { (millimeter) : [20, 55, 100] } as LengthBoundSpec;
const B_PITCH      = { (millimeter) : [50, 84, 180] } as LengthBoundSpec;
const B_CLR        = { (millimeter) : [0, 0.3, 1.5] } as LengthBoundSpec;
const B_SLOT       = { (millimeter) : [2.6, 3.4, 6] } as LengthBoundSpec;
const B_HDR        = { (millimeter) : [3, 8.5, 15] } as LengthBoundSpec;
const B_FOAM       = { (millimeter) : [0, 1.0, 4] } as LengthBoundSpec;
const B_IMU_L      = { (millimeter) : [10, 21.2, 40] } as LengthBoundSpec;
const B_IMU_W      = { (millimeter) : [10, 16.4, 40] } as LengthBoundSpec;
const B_DRV_L      = { (millimeter) : [10, 18.5, 40] } as LengthBoundSpec;
const B_DRV_W      = { (millimeter) : [10, 16.0, 40] } as LengthBoundSpec;
const B_MAG_L      = { (millimeter) : [8, 14.8, 30] } as LengthBoundSpec;
const B_MAG_W      = { (millimeter) : [8, 13.5, 30] } as LengthBoundSpec;
const B_GPS_L      = { (millimeter) : [15, 36, 60] } as LengthBoundSpec;
const B_GPS_W      = { (millimeter) : [15, 25, 40] } as LengthBoundSpec;
const B_ANT_L      = { (millimeter) : [10, 25, 40] } as LengthBoundSpec;
const B_ANT_H      = { (millimeter) : [2, 6.2, 12] } as LengthBoundSpec;
const B_BAT_L      = { (millimeter) : [20, 42, 80] } as LengthBoundSpec;
const B_BAT_W      = { (millimeter) : [10, 20, 40] } as LengthBoundSpec;
const B_BAT_T      = { (millimeter) : [3, 8.5, 20] } as LengthBoundSpec;
const B_BAT_CLR    = { (millimeter) : [0, 0.4, 2] } as LengthBoundSpec;
const B_BAY        = { (millimeter) : [4, 10, 30] } as LengthBoundSpec;
const B_DECK_T     = { (millimeter) : [0.6, 1.0, 4] } as LengthBoundSpec;
const B_ARM_W      = { (millimeter) : [3, 6, 20] } as LengthBoundSpec;
const B_ARM_W_ROOT = { (millimeter) : [3, 12, 30] } as LengthBoundSpec;
const B_WEB_T      = { (millimeter) : [0.8, 1.4, 4] } as LengthBoundSpec;
const B_WEB_D      = { (millimeter) : [2, 10, 25] } as LengthBoundSpec;
const B_WEB_D_TIP  = { (millimeter) : [1, 5, 25] } as LengthBoundSpec;
const B_WALL       = { (millimeter) : [0.8, 1.0, 4] } as LengthBoundSpec;
const B_POD_WALL   = { (millimeter) : [0.8, 1.1, 4] } as LengthBoundSpec;
const B_MAST_TOP   = { (millimeter) : [8, 17, 40] } as LengthBoundSpec;
const B_EDGE_R     = { (millimeter) : [0, 0.4, 1.5] } as LengthBoundSpec;

annotation { "Feature Type Name" : "Pico Drone Frame",
             "Feature Type Description" : "Parametric quad frame for the pico-drone build: frame, GPS/compass mast, tolerance coupon and fit-check dummies." }
export const picoDroneFrame = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        annotation { "Group Name" : "Motors and props", "Collapsed By Default" : false }
        {
            annotation { "Name" : "Motor diameter (720 = 7.0, 8520 = 8.5)" }
            isLength(definition.motorD, B_MOTOR_D);
            annotation { "Name" : "Motor bore allowance (bore = motor dia + this)" }
            isLength(definition.motorFit, B_MOTOR_FIT);
            annotation { "Name" : "Motor can length" }
            isLength(definition.motorLen, B_MOTOR_LEN);
            annotation { "Name" : "Motor top height above deck" }
            isLength(definition.motorRise, B_MOTOR_RISE);
            annotation { "Name" : "Prop diameter" }
            isLength(definition.propD, B_PROP_D);
            annotation { "Name" : "Motor spacing (adjacent, centre to centre)" }
            isLength(definition.pitch, B_PITCH);
        }
        annotation { "Group Name" : "Boards and fit tolerances", "Collapsed By Default" : false }
        {
            annotation { "Name" : "Board pocket clearance (per side)" }
            isLength(definition.clr, B_CLR);
            annotation { "Name" : "Header slot width (strip is 2.54)" }
            isLength(definition.slotW, B_SLOT);
            annotation { "Name" : "Header depth below PCB (plastic + pins)" }
            isLength(definition.hdrBelow, B_HDR);
            annotation { "Name" : "Foam tape thickness (IMU, GPS, compass)" }
            isLength(definition.foam, B_FOAM);
            annotation { "Name" : "GY-521 length (X arrow side)" }
            isLength(definition.imuL, B_IMU_L);
            annotation { "Name" : "GY-521 width (header side)" }
            isLength(definition.imuW, B_IMU_W);
            annotation { "Name" : "GY-521 silkscreen X runs along the short edge" }
            definition.imuRot90 is boolean;
            annotation { "Name" : "DRV8833 length (across the two header rows)" }
            isLength(definition.drvL, B_DRV_L);
            annotation { "Name" : "DRV8833 width (along a header row)" }
            isLength(definition.drvW, B_DRV_W);
            annotation { "Name" : "GY-271 length (along X)" }
            isLength(definition.magL, B_MAG_L);
            annotation { "Name" : "GY-271 width" }
            isLength(definition.magW, B_MAG_W);
            annotation { "Name" : "GPS board length" }
            isLength(definition.gpsL, B_GPS_L);
            annotation { "Name" : "GPS board width" }
            isLength(definition.gpsW, B_GPS_W);
            annotation { "Name" : "GPS antenna side" }
            isLength(definition.antL, B_ANT_L);
            annotation { "Name" : "GPS antenna height" }
            isLength(definition.antH, B_ANT_H);
        }
        annotation { "Group Name" : "Battery", "Collapsed By Default" : false }
        {
            annotation { "Name" : "Battery length (incl. protection PCB)" }
            isLength(definition.batL, B_BAT_L);
            annotation { "Name" : "Battery width" }
            isLength(definition.batW, B_BAT_W);
            annotation { "Name" : "Battery thickness" }
            isLength(definition.batT, B_BAT_T);
            annotation { "Name" : "Battery bay clearance (per side)" }
            isLength(definition.batClr, B_BAT_CLR);
            annotation { "Name" : "Wiring gap under deck (10 soldered, ~18 Dupont)" }
            isLength(definition.bay, B_BAY);
        }
        annotation { "Group Name" : "Structure and finish", "Collapsed By Default" : true }
        {
            annotation { "Name" : "Deck thickness" }
            isLength(definition.deckT, B_DECK_T);
            annotation { "Name" : "Arm flange width at motor" }
            isLength(definition.armW, B_ARM_W);
            annotation { "Name" : "Arm flange width at deck" }
            isLength(definition.armWRoot, B_ARM_W_ROOT);
            annotation { "Name" : "Arm web thickness" }
            isLength(definition.webT, B_WEB_T);
            annotation { "Name" : "Arm web depth at deck" }
            isLength(definition.webD, B_WEB_D);
            annotation { "Name" : "Arm web depth at motor" }
            isLength(definition.webDTip, B_WEB_D_TIP);
            annotation { "Name" : "Wall thickness (bay trusses, rims, mast)" }
            isLength(definition.wallT, B_WALL);
            annotation { "Name" : "Motor pod wall thickness" }
            isLength(definition.podWall, B_POD_WALL);
            annotation { "Name" : "Mast platform top height" }
            isLength(definition.mastTop, B_MAST_TOP);
            annotation { "Name" : "Edge rounding radius (0 = off)" }
            isLength(definition.edgeR, B_EDGE_R);
        }
        annotation { "Name" : "Make GPS + compass mast", "Default" : true }
        definition.makeMast is boolean;
        annotation { "Name" : "Make tolerance test coupon", "Default" : true }
        definition.makeCoupon is boolean;
        annotation { "Name" : "Show fit-check dummies (boards, battery, motors, props)", "Default" : true }
        definition.showFit is boolean;
    }
    {
        // ------------------------------------------------------------ inputs as plain mm numbers
        const mD = definition.motorD / MM;
        const mFit = definition.motorFit / MM;
        const mLen = definition.motorLen / MM;
        const mRise = definition.motorRise / MM;
        const propD = definition.propD / MM;
        const a = definition.pitch / MM / 2;          // motor centres at (+-a, +-a)
        const clr = definition.clr / MM;
        const slotW = definition.slotW / MM;
        const hdr = definition.hdrBelow / MM;
        const foam = definition.foam / MM;
        const imuX = definition.imuRot90 ? definition.imuW / MM : definition.imuL / MM;  // along X
        const imuY = definition.imuRot90 ? definition.imuL / MM : definition.imuW / MM;  // along Y
        const drvL = definition.drvL / MM;
        const drvW = definition.drvW / MM;
        const magL = definition.magL / MM;
        const magW = definition.magW / MM;
        const gpsL = definition.gpsL / MM;
        const gpsW = definition.gpsW / MM;
        const antL = definition.antL / MM;
        const antH = definition.antH / MM;
        const batL = definition.batL / MM;
        const batW = definition.batW / MM;
        const batT = definition.batT / MM;
        const batClr = definition.batClr / MM;
        const bay = definition.bay / MM;
        const T = definition.deckT / MM;
        const armW = definition.armW / MM;
        const armWRoot = definition.armWRoot / MM;
        const webT = definition.webT / MM;
        const webD = definition.webD / MM;
        const webDTip = definition.webDTip / MM;
        const wallT = definition.wallT / MM;
        const podWall = definition.podWall / MM;
        const mastTop = definition.mastTop / MM;
        const edgeR = definition.edgeR / MM;

        // ------------------------------------------------------------ Pico 2 W (datasheet, fixed)
        const picoL = 51.0;
        const picoW = 21.0;
        const picoRowDY = 8.89;       // 17.78 / 2
        const picoStrip = 50.8;       // 20 pins x 2.54
        const usbOver = 1.3;

        // ------------------------------------------------------------ layout
        // Two columns side by side: Pico on the left (+Y), and DRV1 / IMU / DRV2
        // on the right (-Y), front to back. Battery hangs under the middle.
        const pkW = picoW + 2 * clr;
        const rcW = max(drvL, imuY) + 2 * clr;
        const colGap = 2.0;
        const totW = pkW + colGap + rcW;
        const yMin = -totW / 2;
        const rcY = yMin + rcW / 2;
        const picoY = yMin + rcW + colGap + pkW / 2;
        const DY = totW / 2 + 1.6;

        const drvPx = drvW + 2 * clr;
        const imuPx = imuX + 2 * clr;
        const cellGap = 1.6;
        const rcLen = 2 * drvPx + imuPx + 2 * cellGap;
        const drv1X = rcLen / 2 - drvPx / 2;          // front driver (M1, M2)
        const drv2X = -drv1X;                          // rear driver (M3, M4)
        const xBoards = max(rcLen / 2, picoL / 2 + clr);

        const legT = 1.6;
        const mSlotW = legT + 0.3;
        const xs = xBoards + 0.6 + mSlotW / 2;         // mast leg slot centre
        const DX = xs + mSlotW / 2 + 1.6;
        const Rc = 8.0;                                // deck corner radius (keeps corners out of the prop wash)

        // motors and battery
        const zMb = mRise - mLen;                      // motor bottom
        const rb = (mD + mFit) / 2;                    // pod bore radius
        const ro = rb + podWall;                       // pod outer radius
        const rl = rb - 1.0;                           // bottom-lip opening radius
        const zPod = zMb - 1.6;                        // pod bottom
        const cl = batL / 2 + batClr;                  // battery cavity half length
        const cw = batW / 2 + batClr;                  // battery cavity half width
        const kx = cl + wallT;                         // bay wall half length
        const zBt = -bay;                              // battery top
        const zBb = zBt - (batT + batClr);             // battery bottom
        const zFl = zBb - wallT;                       // floor underside
        const rimD = 3.0;                              // side rim depth below deck top
        const legY0 = -12.0;
        const legY1 = 2.0;
        const screwY = -9.0;
        const screwZ = -4.2;
        const tabBot = -6.5;
        const gT = max(wallT, webT);                   // gusset thickness - wide enough to swallow the web root
        const armRootX = DY - gT / 2;

        // every operation gets its own id; the registry remembers which part it belongs to
        const reg = new box({ "n" : 0, "base" : id });

        // ============================================================ FRAME
        // deck + all four arm flanges as ONE rounded outline: the joints between
        // deck and arms are blended, and the top face has a single smooth edge loop
        const rOut = a * sqrt(2);
        const rIn = (DY - 5) * sqrt(2);
        const rWeb = armRootX * sqrt(2);
        var outline = [];
        for (var qd in [[1, 1, 45], [-1, 1, 135], [-1, -1, -135], [1, -1, -45]])
        {
            const qsx = qd[0];
            const qsy = qd[1];
            const qc = cos(qd[2] * degree);
            const qs = sin(qd[2] * degree);
            const outE = exitRect(armEdgePts(qc, qs, rIn, rWeb, rOut, armWRoot / 2, armW / 2, -1), DX, DY);
            const retE = exitRect(armEdgePts(qc, qs, rIn, rWeb, rOut, armWRoot / 2, armW / 2, 1), DX, DY);
            const endIn = qsx * qsy > 0;                 // arriving along the front/back edge
            if (endIn)
                outline = append(outline, [qsx * DX, qsy * (DY - Rc), 4.0]);
            outline = append(outline, [outE[0][0], outE[0][1], 3.0]);
            for (var k = 1; k < size(outE); k += 1)
                outline = append(outline, [outE[k][0], outE[k][1], 1.5]);
            for (var k = size(retE) - 1; k >= 1; k -= 1)
                outline = append(outline, [retE[k][0], retE[k][1], 1.5]);
            outline = append(outline, [retE[0][0], retE[0][1], 3.0]);
            if (!endIn)
                outline = append(outline, [qsx * DX, qsy * (DY - Rc), 4.0]);
        }
        prismR(context, nid(reg, "fA"), outline, 0, -T, 0);

        // side rims along the long edges, ends buried inside the arm-root gussets
        for (var sy in [-1, 1])
        {
            const y0 = sy > 0 ? DY - wallT : -DY;
            box3(context, nid(reg, "fA"), [-(armRootX + 1.0), y0, -rimD], [armRootX + 1.0, y0 + wallT, 0]);
        }

        // arm-root gussets: carry each arm's bending into the deck edge
        for (var sx in [-1, 1])
        {
            for (var sy in [-1, 1])
            {
                const yc = sy * (DY - gT / 2);
                const xr = sx * armRootX;
                const xEnd = sx * min(armRootX + 2.9, DX - Rc - 0.2);
                prismPlaneR(context, nid(reg, "fA"), plane(v3(0, yc + gT / 2, 0), vector(0, -1, 0), vector(1, 0, 0)),
                    [[xr - sx * 11, 0.0, 0], [xEnd, 0.0, 0], [xr + sx * 1.0, -T - webD - 1.0, 0.8], [xr - sx * 2.0, -T - webD - 1.0, 0.8]], 0, gT);
            }
        }

        // battery bay walls (Warren trusses) + 45 deg hold-down rails
        for (var sy in [-1, 1])
        {
            const yHi = sy > 0 ? cw + wallT : -cw;
            const pl = plane(v3(0, yHi, 0), vector(0, -1, 0), vector(1, 0, 0));
            prismPlane(context, nid(reg, "fA"), pl, [[-kx, 0.0], [-kx, zFl], [kx, zFl], [kx, 0.0]], wallT);
            prismPlane(context, nid(reg, "fA"),
                plane(v3(-cl, 0, 0), vector(1, 0, 0), vector(0, 1, 0)),
                [[sy * cw, zBt], [sy * (cw - 1.5), zBt], [sy * cw, zBt + 1.5]], 2 * cl);
            // truss windows through the wall
            const plCut = plane(v3(0, yHi + 0.5, 0), vector(0, -1, 0), vector(1, 0, 0));
            for (var tri in warren(-kx + 1.5, kx - 1.5, zFl + 2.0, zBt - 0.6, 0.9))
                prismPlaneR(context, nid(reg, "fC"), plCut, tri, 0.8, wallT + 1);
            if (bay > 6)
            {
                for (var tri in warren(-kx + 1.5, kx - 1.5, zBt + 2.3, -T - 1.4, 0.9))
                    prismPlaneR(context, nid(reg, "fC"), plCut, tri, 0.8, wallT + 1);
            }
        }
        // battery floor bars and front stop
        for (var fx in [-(cl - 6), cl - 6])
            prismR(context, nid(reg, "fA"), rrect(fx - 2.5, -cw - wallT, fx + 2.5, cw + wallT), 1.0, zFl, zBb);
        prismR(context, nid(reg, "fA"), rrect(cl, -cw - wallT, cl + wallT, cw + wallT), wallT / 2, zFl, zBb + 4.0);

        // arms: tapered T-section - flange in the deck plane, web hanging below,
        // both biggest at the deck where the bending moment is largest
        const arms = [[1, -1, "M1"], [-1, 1, "M2"], [1, 1, "M3"], [-1, -1, "M4"]];
        for (var arm in arms)
        {
            const sx = arm[0];
            const sy = arm[1];
            const ang = sx > 0 ? (sy > 0 ? 45 : -45) : (sy > 0 ? 135 : -135);
            const c = cos(ang * degree);
            const s = sin(ang * degree);
            prismPlaneR(context, nid(reg, "fA"),
                plane(v3(-webT / 2 * s, webT / 2 * c, 0), vector(s, -c, 0), vector(c, s, 0)),
                [[rWeb, -T + 0.01, 0], [rOut, -T + 0.01, 0], [rOut, -T - webDTip, 1.5], [rWeb, -T - webD, 1.5]], 0, webT);
            // motor pod
            zcyl(context, nid(reg, "fA"), a * sx, a * sy, zPod, 0, ro);

            // pod bore, 45 deg bottom lip (wire exit), clamp slit on the outboard side
            zcyl(context, nid(reg, "fC"), a * sx, a * sy, zMb, 1, rb);
            fCone(context, nid(reg, "fC"), {
                        "bottomCenter" : v3(a * sx, a * sy, zMb - 1.0), "bottomRadius" : rl * MM,
                        "topCenter" : v3(a * sx, a * sy, zMb + 0.001), "topRadius" : (rb + 0.001) * MM });
            zcyl(context, nid(reg, "fC"), a * sx, a * sy, zPod - 1, zMb - 0.5, rl);
            const sMid = rOut + (ro + 1) / 2;
            prismZ(context, nid(reg, "fC"), rrectPts(sMid * c, sMid * s, ro + 1, 0.8, ang), zPod - 1, 1);
            // wire-tie holes through the web
            for (var f in [0.3, 0.65])
            {
                const r = rWeb + f * (rOut - rWeb);
                const hz = -T - (webD + f * (webDTip - webD)) / 2;
                cylAxis(context, nid(reg, "fC"), [r * c + 2 * s, r * s - 2 * c, hz], [r * c - 2 * s, r * s + 2 * c, hz], 1.0);
            }
            // label + nose arrow are engraved after the edge rounding
            const lr = (rWeb + rOut) / 2 - 1;
            try silent
            {
                engraveText(context, nid(reg, "fE"), arm[2], lr * c, lr * s, 6.0, 3.4);
            }
            if (sx > 0)
            {
                const ar = rWeb + 4.5;
                const ax = ar * c;
                const ay = ar * s;
                prismZ(context, nid(reg, "fE"), [[ax + 2.2, ay], [ax - 1.6, ay + 2.0], [ax - 1.6, ay - 2.0]], -0.4, 1);
            }
        }

        // ---- header slots (pins-down strips drop through the deck); small corner radii,
        // because fully round ends would clip the square corners of the strips
        for (var sy in [-1, 1])
        {
            const yr = picoY + sy * picoRowDY;
            prismR(context, nid(reg, "fC"), rrect(-picoStrip / 2 - 0.3, yr - slotW / 2, picoStrip / 2 + 0.3, yr + slotW / 2), 0.6, -T - 1, 1);
        }
        // USB plug clearance at the rear of the Pico
        prismR(context, nid(reg, "fC"), rrect(-DX - 3, picoY - 6.5, -picoL / 2 - 0.6, picoY + 6.5), 1.5, -T - webD - 2, 1);

        // DRV8833 x2: two 6-pin rows each, rows run along X
        const drvRowDY = drvL / 2 - 1.63;
        const dSlot = slotW + 0.4;
        for (var dx in [drv1X, drv2X])
        {
            for (var sy in [-1, 1])
            {
                const yr = rcY + sy * drvRowDY;
                prismR(context, nid(reg, "fC"), rrect(dx - drvW / 2, yr - dSlot / 2, dx + drvW / 2, yr + dSlot / 2), 0.6, -T - 1, 1);
            }
        }
        // GY-521: one 8-pin row on a long edge - slot both edges so the board
        // can be turned 180 deg to match the silkscreen X arrow
        const imuStrip = 8 * 2.54 + 0.6;
        const imuRowD = (definition.imuRot90 ? imuX : imuY) / 2 - 1.27;
        for (var sy in [-1, 1])
        {
            if (definition.imuRot90)
                prismR(context, nid(reg, "fC"), rrect(sy * imuRowD - slotW / 2, rcY - imuStrip / 2, sy * imuRowD + slotW / 2, rcY + imuStrip / 2), 0.6, -T - 1, 1);
            else
                prismR(context, nid(reg, "fC"), rrect(-imuStrip / 2, rcY + sy * imuRowD - slotW / 2, imuStrip / 2, rcY + sy * imuRowD + slotW / 2), 0.6, -T - 1, 1);
        }

        // ---- lightening windows under the boards, kept clear of the bay-wall line
        const kLo = cw - 0.8;                // deck must stay solid over y in [kLo, kHi] (and mirrored)
        const kHi = cw + wallT + 0.8;
        // Pico, between its rows
        const pA0 = picoY - picoRowDY + slotW / 2 + 1.2;
        const pB1 = picoY + picoRowDY - slotW / 2 - 1.2;
        for (var band in splitBand(pA0, pB1, kLo, kHi))
            prismR(context, nid(reg, "fC"), rrect(-21.3, band[0], 21.3, band[1]), 1.2, -T - 1, 1);
        // DRV islands
        for (var dx in [drv1X, drv2X])
        {
            const i0 = rcY - drvRowDY + dSlot / 2 + 1.2;
            const i1 = rcY + drvRowDY - dSlot / 2 - 1.2;
            for (var band in splitBand(-i1, -i0, kLo, kHi))
                prismR(context, nid(reg, "fC"), rrect(dx - drvW / 2 + 1.6, -band[1], dx + drvW / 2 - 1.6, -band[0]), 1.2, -T - 1, 1);
        }
        // IMU island (foam tape bridges the window)
        if (!definition.imuRot90)
        {
            const j0 = rcY - imuRowD + slotW / 2 + 1.2;
            const j1 = rcY + imuRowD - slotW / 2 - 1.2;
            for (var band in splitBand(-j1, -j0, kLo, kHi))
                prismR(context, nid(reg, "fC"), rrect(-imuX / 2 + 2.0, -band[1], imuX / 2 - 2.0, -band[0]), 1.2, -T - 1, 1);
        }

        // ---- mast sockets: slots for the leg tabs, screw blocks under the deck
        if (definition.makeMast)
        {
            for (var sx in [-1, 1])
            {
                prismR(context, nid(reg, "fC"), rrect(sx * xs - mSlotW / 2, legY0 - 0.3, sx * xs + mSlotW / 2, legY1 + 0.3), 0.45, -T - 1, 1);
                const inner = sx * (xs - mSlotW / 2);
                box3(context, nid(reg, "fA"), [inner - sx * 0.2, legY0, tabBot], [inner - sx * 3.2, -6.6, 0]);
                box3(context, nid(reg, "fA"), [sx * DX, -13, tabBot], [sx * (DX - wallT - 0.2), -5, 0]);
                cylAxis(context, nid(reg, "fC"), [sx * (DX + 1), screwY, screwZ], [sx * (xs + mSlotW / 2 - 0.01), screwY, screwZ], 1.1);
                cylAxis(context, nid(reg, "fC"), [inner + sx * 0.5, screwY, screwZ], [inner - sx * 3.7, screwY, screwZ], 0.85);
            }
        }

        const frameQ = finishPart(context, reg, "fA", "fC");
        const frameRound = roundConvexEdges(context, reg, frameQ, edgeR);
        cutKind(context, reg, frameQ, "fE");
        setProperty(context, { "entities" : frameQ, "propertyType" : PropertyType.NAME, "value" : "Frame (print deck-down)" });
        setProperty(context, { "entities" : frameQ, "propertyType" : PropertyType.APPEARANCE, "value" : color(0.15, 0.15, 0.17) });
        setProperty(context, { "entities" : frameQ, "propertyType" : PropertyType.MATERIAL, "value" : material("PLA", 1.24 * gram / centimeter ^ 3) });

        var info = "Frame " ~ fmt1(massOf(context, frameQ, 1.24)) ~ " g";
        var parts = [[frameQ, massOf(context, frameQ, 1.24)]];
        var rounding = "frame " ~ frameRound;

        // ============================================================ MAST
        const platT = 1.0;
        const platY = gpsW / 2 + clr + 0.6;
        const magX = magL + 2 * clr;
        const gpsX = gpsL + 2 * clr;
        const stackLen = magX + 3 + gpsX;
        const magCX = -stackLen / 2 + magX / 2;
        const gpsCX = stackLen / 2 - gpsX / 2;
        const zPl = mastTop - platT;                   // platform underside
        if (definition.makeMast)
        {
            const px = xs + legT / 2 + 1.5;
            prismR(context, nid(reg, "mA"), rrect(-px, -platY, px, platY), 2.5, zPl, mastTop);
            // stiffening lips under the long edges, stopping short of the legs
            const lipX = xs - legT / 2 - 1.0;
            for (var sy in [-1, 1])
            {
                const y0 = sy > 0 ? platY - wallT : -platY;
                box3(context, nid(reg, "mA"), [-lipX, y0, zPl - 2.0], [lipX, y0 + wallT, zPl]);
            }
            for (var sx in [-1, 1])
            {
                // leg + tab drawn in the YZ plane: shoulders rest on the deck, tab drops through the slot
                const plLeg = plane(v3(sx * xs - legT / 2, 0, 0), vector(1, 0, 0), vector(0, 1, 0));
                prismPlane(context, nid(reg, "mA"), plLeg,
                    [[legY0, tabBot], [legY1, tabBot], [legY1, 0], [legY1 + 1, 0], [legY1 + 1, zPl], [legY0 - 1, zPl], [legY0 - 1, 0], [legY0, 0]], legT);
                const plLegCut = plane(v3(sx * xs - legT / 2 - 0.5, 0, 0), vector(1, 0, 0), vector(0, 1, 0));
                for (var tri in warren(legY0 + 0.2, legY1 - 0.2, 2.0, zPl - 2.0, 0.8))
                    prismPlaneR(context, nid(reg, "mC"), plLegCut, tri, 0.7, legT + 1);
                cylAxis(context, nid(reg, "mC"), [sx * xs - 2, screwY, screwZ], [sx * xs + 2, screwY, screwZ], 1.1);
            }
            // GPS: 4-pin row on a short edge - slots at both short edges
            for (var sx in [-1, 1])
            {
                const rx = gpsCX + sx * (gpsL / 2 - 1.27);
                prismR(context, nid(reg, "mC"), rrect(rx - slotW / 2, -10, rx + slotW / 2, 10), 0.6, zPl - 1, mastTop + 1);
            }
            // GY-271: 5-pin row on a front/back edge - slots at both
            const magStrip = 5 * 2.54 + 0.6;
            for (var sx in [-1, 1])
            {
                const rx = magCX + sx * (magL / 2 - 1.27);
                prismR(context, nid(reg, "mC"), rrect(rx - slotW / 2, -magStrip / 2, rx + slotW / 2, magStrip / 2), 0.6, zPl - 1, mastTop + 1);
            }
            // windows under both boards; foam tape sits on the strips that remain
            prismR(context, nid(reg, "mC"), rrect(gpsCX - gpsL / 2 + 5.5, -(gpsW / 2 - 4.0), gpsCX + gpsL / 2 - 5.5, gpsW / 2 - 4.0), 2.0, zPl - 1, mastTop + 1);
            if (magL > 10 && magW > 9)
                prismR(context, nid(reg, "mC"), rrect(magCX - magL / 2 + 4.5, -(magW / 2 - 3.2), magCX + magL / 2 - 4.5, magW / 2 - 3.2), 1.5, zPl - 1, mastTop + 1);

            const mastQ = finishPart(context, reg, "mA", "mC");
            rounding = rounding ~ ", mast " ~ roundConvexEdges(context, reg, mastQ, edgeR);
            setProperty(context, { "entities" : mastQ, "propertyType" : PropertyType.NAME, "value" : "GPS + compass mast (print platform-down)" });
            setProperty(context, { "entities" : mastQ, "propertyType" : PropertyType.APPEARANCE, "value" : color(0.95, 0.45, 0.1) });
            setProperty(context, { "entities" : mastQ, "propertyType" : PropertyType.MATERIAL, "value" : material("PLA", 1.24 * gram / centimeter ^ 3) });
            info = info ~ " - mast " ~ fmt1(massOf(context, mastQ, 1.24)) ~ " g";
            parts = append(parts, [mastQ, massOf(context, mastQ, 1.24)]);
        }

        // ============================================================ COUPON
        if (definition.makeCoupon)
        {
            const cy0 = DY + 40;
            const allow = [-0.1, 0.0, 0.1, 0.2, 0.3];
            prismR(context, nid(reg, "kA"), rrect(-30, cy0, 30, cy0 + 24), 3.0, -T, 0);
            for (var i = 0; i < size(allow); i += 1)
            {
                const cx = -24 + i * 12;
                const r = (mD + allow[i]) / 2;
                zcyl(context, nid(reg, "kA"), cx, cy0 + 1, -8, 0, r + podWall);
                zcyl(context, nid(reg, "kC"), cx, cy0 + 1, -9, 1, r);
                prismZ(context, nid(reg, "kC"), [[cx - 0.4, cy0 + 1 - r - podWall - 1], [cx + 0.4, cy0 + 1 - r - podWall - 1], [cx + 0.4, cy0 + 1], [cx - 0.4, cy0 + 1]], -9, 1);
            }
            const slots = [3.0, 3.4, 3.8];
            for (var i = 0; i < size(slots); i += 1)
            {
                const sxc = -20 + i * 14;
                prismR(context, nid(reg, "kC"), rrect(sxc - 6, cy0 + 18 - slots[i] / 2, sxc + 6, cy0 + 18 + slots[i] / 2), 0.6, -T - 1, 1);
            }
            const cpnQ = finishPart(context, reg, "kA", "kC");
            rounding = rounding ~ ", coupon " ~ roundConvexEdges(context, reg, cpnQ, edgeR);
            // i+1 dots tell the bore allowances apart: -0.1, 0.0, +0.1, +0.2, +0.3
            for (var i = 0; i < size(allow); i += 1)
                for (var j = 0; j <= i; j += 1)
                    zcyl(context, nid(reg, "kE"), -28 + i * 12 + j * 2, cy0 + 9.0, -0.4, 1, 0.5);
            cutKind(context, reg, cpnQ, "kE");
            setProperty(context, { "entities" : cpnQ, "propertyType" : PropertyType.NAME, "value" : "Tolerance coupon (print first)" });
            setProperty(context, { "entities" : cpnQ, "propertyType" : PropertyType.APPEARANCE, "value" : color(0.2, 0.6, 0.9) });
        }

        // ============================================================ FIT-CHECK DUMMIES
        // Each dummy is a list of shapes: ["B", corner, corner] or ["C", x, y, z0, z1, r].
        // They carry the real component masses so the summary can report all-up weight and CG.
        if (definition.showFit)
        {
            const e = 0.05;   // dummies float this far off their seats, so touching never reads as overlap
            // Pico 2 W
            var sp = [["B", [-picoL / 2, picoY - picoW / 2, e], [picoL / 2, picoY + picoW / 2, e + 1]],
                      ["B", [-picoL / 2 - usbOver, picoY - 4, e + 1], [-picoL / 2 + 4.2, picoY + 4, e + 3.7]],
                      ["B", [-4, picoY - 4, e + 1], [4, picoY + 4, e + 1.9]]];
            for (var sy in [-1, 1])
            {
                sp = append(sp, ["B", [-picoStrip / 2, picoY + sy * picoRowDY - 1.27, e - 2.5], [picoStrip / 2, picoY + sy * picoRowDY + 1.27, e]]);
                sp = append(sp, ["B", [-picoStrip / 2 + 1, picoY + sy * picoRowDY - 0.32, e - hdr], [picoStrip / 2 - 1, picoY + sy * picoRowDY + 0.32, e - 2.5]]);
            }
            parts = append(parts, dummy(context, reg, "fitPico", sp, "FIT - Pico 2 W", color(0.1, 0.55, 0.2), 5.5));

            // DRV8833 x2
            for (var k = 0; k < 2; k += 1)
            {
                const dx = k == 0 ? drv1X : drv2X;
                var sd = [["B", [dx - drvW / 2, rcY - drvL / 2, e], [dx + drvW / 2, rcY + drvL / 2, e + 1.6]],
                          ["B", [dx - 3, rcY - 3, e + 1.6], [dx + 3, rcY + 3, e + 2.8]]];
                for (var sy in [-1, 1])
                {
                    sd = append(sd, ["B", [dx - 7.62, rcY + sy * drvRowDY - 1.27, e - 2.5], [dx + 7.62, rcY + sy * drvRowDY + 1.27, e]]);
                    sd = append(sd, ["B", [dx - 6.6, rcY + sy * drvRowDY - 0.32, e - hdr], [dx + 6.6, rcY + sy * drvRowDY + 0.32, e - 2.5]]);
                }
                parts = append(parts, dummy(context, reg, "fitDrv" ~ k, sd, "FIT - DRV8833 #" ~ (k + 1), color(0.8, 0.1, 0.1), 1.5));
            }

            // GY-521 on foam tape
            const z0i = foam + e;
            var si = [["B", [-imuX / 2, rcY - imuY / 2, z0i], [imuX / 2, rcY + imuY / 2, z0i + 1.6]],
                      ["B", [-2, rcY - 2, z0i + 1.6], [2, rcY + 2, z0i + 2.5]]];
            if (definition.imuRot90)
            {
                const hx = -(imuX / 2 - 1.27);
                si = append(si, ["B", [hx - 1.27, rcY - 10.16, z0i - 2.5], [hx + 1.27, rcY + 10.16, z0i]]);
                si = append(si, ["B", [hx - 0.32, rcY - 9.2, z0i - hdr], [hx + 0.32, rcY + 9.2, z0i - 2.5]]);
            }
            else
            {
                const hy = rcY - (imuY / 2 - 1.27);
                si = append(si, ["B", [-10.16, hy - 1.27, z0i - 2.5], [10.16, hy + 1.27, z0i]]);
                si = append(si, ["B", [-9.2, hy - 0.32, z0i - hdr], [9.2, hy + 0.32, z0i - 2.5]]);
            }
            parts = append(parts, dummy(context, reg, "fitImu", si, "FIT - GY-521 IMU", color(0.15, 0.3, 0.85), 2.0));

            // battery
            parts = append(parts, dummy(context, reg, "fitBat",
                        [["B", [-batL / 2, -batW / 2, zBb + batClr / 2], [batL / 2, batW / 2, zBb + batClr / 2 + batT]]],
                        "FIT - 852040 battery", color(0.75, 0.75, 0.78), 12.5));

            // motors + props
            for (var arm in arms)
            {
                const mx = a * arm[0];
                const my = a * arm[1];
                parts = append(parts, dummy(context, reg, "fitMot" ~ arm[2],
                            [["C", mx, my, zMb + e, mRise, mD / 2], ["C", mx, my, mRise, mRise + 5, 0.5]],
                            "FIT - motor " ~ arm[2], color(0.6, 0.6, 0.62), 3.4));
                parts = append(parts, dummy(context, reg, "fitProp" ~ arm[2],
                            [["C", mx, my, mRise + 0.6, mRise + 5.6, 2.6], ["C", mx, my, mRise + 1.5, mRise + 4.5, propD / 2]],
                            "FIT - prop disc " ~ arm[2], color(0.95, 0.85, 0.2), 0.6));
            }

            if (definition.makeMast)
            {
                // GPS board + module + patch antenna on foam tape, on the mast
                const z0g = mastTop + foam + e;
                const hxg = gpsCX + (gpsL / 2 - 1.27);
                parts = append(parts, dummy(context, reg, "fitGps",
                            [["B", [gpsCX - gpsL / 2, -gpsW / 2, z0g], [gpsCX + gpsL / 2, gpsW / 2, z0g + 1.6]],
                             ["B", [gpsCX - 8, -6.1, z0g + 1.6], [gpsCX + 8, 6.1, z0g + 4.0]],
                             ["B", [gpsCX - antL / 2, -antL / 2, z0g + 4.0], [gpsCX + antL / 2, antL / 2, z0g + 5.0 + antH]],
                             ["B", [hxg - 1.27, -5.08, z0g - 2.5], [hxg + 1.27, 5.08, z0g]],
                             ["B", [hxg - 0.32, -4.2, z0g - hdr], [hxg + 0.32, 4.2, z0g - 2.5]]],
                            "FIT - GY-GPS6MV2 + antenna", color(0.2, 0.2, 0.2), 14.0));
                const hxm = magCX - (magL / 2 - 1.27);
                parts = append(parts, dummy(context, reg, "fitMag",
                            [["B", [magCX - magL / 2, -magW / 2, z0g], [magCX + magL / 2, magW / 2, z0g + 1.6]],
                             ["B", [magCX - 1.5, -1.5, z0g + 1.6], [magCX + 1.5, 1.5, z0g + 2.5]],
                             ["B", [hxm - 1.27, -6.35, z0g - 2.5], [hxm + 1.27, 6.35, z0g]],
                             ["B", [hxm - 0.32, -5.5, z0g - hdr], [hxm + 0.32, 5.5, z0g - 2.5]]],
                            "FIT - GY-271 compass", color(0.55, 0.2, 0.7), 1.3));
            }
        }

        // ------------------------------------------------------------ summary in the feature dialog
        var mTot = 0;
        var cx = 0;
        var cy = 0;
        for (var p in parts)
        {
            const c = evApproximateCentroid(context, { "entities" : p[0] });
            mTot += p[1];
            cx += p[1] * c[0] / MM;
            cy += p[1] * c[1] / MM;
        }
        if (definition.showFit)
        {
            info = info ~ " - all-up (no wires) " ~ fmt1(mTot) ~ " g - CG offset x " ~ fmt1(cx / mTot) ~ ", y " ~ fmt1(cy / mTot) ~ " mm";
        }
        reportFeatureInfo(context, id, info ~ " - rounded: " ~ rounding);
    });

// ================================================================ helpers

function v3(x is number, y is number, z is number) returns Vector
{
    return vector(x, y, z) * MM;
}

// fresh operation id, recorded under kind so the bodies can be found again
function nid(reg is box, kind is string) returns Id
{
    var r = reg[];
    r.n = r.n + 1;
    const newId = r.base + ("o" ~ r.n);
    var lst = r[kind];
    if (lst == undefined)
        lst = [];
    r[kind] = append(lst, newId);
    reg[] = r;
    return newId;
}

// every solid created by operations recorded under kind
function qKind(reg is box, kind is string) returns Query
{
    var qs = [];
    const lst = reg[][kind];
    if (lst != undefined)
    {
        for (var i in lst)
            qs = append(qs, qCreatedBy(i, EntityType.BODY));
    }
    return qBodyType(qUnion(qs), BodyType.SOLID);
}

function box3(context is Context, id is Id, p is array, q is array)
{
    fCuboid(context, id, {
                "corner1" : v3(min(p[0], q[0]), min(p[1], q[1]), min(p[2], q[2])),
                "corner2" : v3(max(p[0], q[0]), max(p[1], q[1]), max(p[2], q[2])) });
}

function zcyl(context is Context, id is Id, x is number, y is number, z0 is number, z1 is number, r is number)
{
    fCylinder(context, id, { "bottomCenter" : v3(x, y, z0), "topCenter" : v3(x, y, z1), "radius" : r * MM });
}

function cylAxis(context is Context, id is Id, p is array, q is array, r is number)
{
    fCylinder(context, id, { "bottomCenter" : v3(p[0], p[1], p[2]), "topCenter" : v3(q[0], q[1], q[2]), "radius" : r * MM });
}

// one edge of an arm flange, root to tip: side = -1 / +1 for the two edges
function armEdgePts(c is number, s is number, rIn is number, rWeb is number, rOut is number, wr is number, wt is number, side is number) returns array
{
    var pts = [];
    for (var uv in [[rIn, side * wr], [rWeb, side * wr], [rOut, side * wt]])
        pts = append(pts, [uv[0] * c - uv[1] * s, uv[0] * s + uv[1] * c]);
    return pts;
}

// walk a polyline outward and cut it where it leaves the rectangle |x| <= hx, |y| <= hy:
// returns [exit point, every later point]
function exitRect(poly is array, hx is number, hy is number) returns array
{
    for (var i = 0; i < size(poly) - 1; i += 1)
    {
        const p = poly[i];
        const q = poly[i + 1];
        const pin = abs(p[0]) <= hx && abs(p[1]) <= hy;
        const qin = abs(q[0]) <= hx && abs(q[1]) <= hy;
        if (pin && !qin)
        {
            var tBest = 1;
            for (var lim in [[0, hx], [0, -hx], [1, hy], [1, -hy]])
            {
                const d = q[lim[0]] - p[lim[0]];
                if (abs(d) > 1e-9)
                {
                    const t = (lim[1] - p[lim[0]]) / d;
                    if (t >= 0 && t < tBest)
                        tBest = t;
                }
            }
            var out = [[p[0] + tBest * (q[0] - p[0]), p[1] + tBest * (q[1] - p[1])]];
            for (var j = i + 1; j < size(poly); j += 1)
                out = append(out, poly[j]);
            return out;
        }
    }
    throw regenError("Arm root is outside the deck - increase motor spacing");
}

// axis-aligned rectangle as a CCW point list
function rrect(x0 is number, y0 is number, x1 is number, y1 is number) returns array
{
    return [[min(x0, x1), min(y0, y1)], [max(x0, x1), min(y0, y1)], [max(x0, x1), max(y0, y1)], [min(x0, x1), max(y0, y1)]];
}

// rectangle centred at (cx, cy), len along angle angDeg, wid across it
function rrectPts(cx is number, cy is number, len is number, wid is number, angDeg is number) returns array
{
    const c = cos(angDeg * degree);
    const s = sin(angDeg * degree);
    const hl = len / 2;
    const hw = wid / 2;
    return [[cx + c * hl - s * hw, cy + s * hl + c * hw],
            [cx - c * hl - s * hw, cy - s * hl + c * hw],
            [cx - c * hl + s * hw, cy - s * hl - c * hw],
            [cx + c * hl + s * hw, cy + s * hl - c * hw]];
}

// the parts of [lo, hi] that avoid [kLo, kHi] and its mirror [-kHi, -kLo], each at least 2 mm wide
function splitBand(lo is number, hi is number, kLo is number, kHi is number) returns array
{
    var bands = [[lo, hi]];
    for (var k in [[kLo, kHi], [-kHi, -kLo]])
    {
        var next = [];
        for (var b in bands)
        {
            if (b[1] <= k[0] || b[0] >= k[1])
                next = append(next, b);
            else
            {
                if (k[0] - b[0] > 0)
                    next = append(next, [b[0], k[0]]);
                if (b[1] - k[1] > 0)
                    next = append(next, [k[1], b[1]]);
            }
        }
        bands = next;
    }
    var out = [];
    for (var b in bands)
    {
        if (b[1] - b[0] >= 2.0)
            out = append(out, b);
    }
    return out;
}

// Warren-truss windows for a panel [u0, u1] x [v0, v1]: alternating up/down
// triangles, each shrunk by w so a strut of width 2w remains between them
function warren(u0 is number, u1 is number, v0 is number, v1 is number, w is number) returns array
{
    const h = v1 - v0;
    if (h < 3 || u1 - u0 < 3)
        return [];
    const n = max(1, round((u1 - u0) / (1.15 * h)));
    const p = (u1 - u0) / n;
    var tris = [];
    for (var k = 0; k < n; k += 1)
    {
        const xa = u0 + k * p;
        tris = append(tris, insetTri([xa, v0], [xa + p, v0], [xa + p / 2, v1], w));
        if (k < n - 1)
            tris = append(tris, insetTri([xa + p / 2, v1], [xa + 1.5 * p, v1], [xa + p, v0], w));
    }
    var out = [];
    for (var t in tris)
    {
        if (size(t) == 3)
            out = append(out, t);
    }
    return out;
}

// shrink a triangle about its incentre so every side moves in by w
function insetTri(A is array, B is array, C is array, w is number) returns array
{
    const la = sqrt((B[0] - C[0]) ^ 2 + (B[1] - C[1]) ^ 2);
    const lb = sqrt((A[0] - C[0]) ^ 2 + (A[1] - C[1]) ^ 2);
    const lc = sqrt((A[0] - B[0]) ^ 2 + (A[1] - B[1]) ^ 2);
    const per = la + lb + lc;
    const ix = (la * A[0] + lb * B[0] + lc * C[0]) / per;
    const iy = (la * A[1] + lb * B[1] + lc * C[1]) / per;
    const area = abs((B[0] - A[0]) * (C[1] - A[1]) - (C[0] - A[0]) * (B[1] - A[1])) / 2;
    const rho = 2 * area / per;
    if (rho - w < 0.6)
        return [];
    const f = (rho - w) / rho;
    return [[ix + (A[0] - ix) * f, iy + (A[1] - iy) * f],
            [ix + (B[0] - ix) * f, iy + (B[1] - iy) * f],
            [ix + (C[0] - ix) * f, iy + (C[1] - iy) * f]];
}

// closed polygon in the XY plane, extruded from z0 up to z1
function prismZ(context is Context, id is Id, pts is array, z0 is number, z1 is number)
{
    prismPlane(context, id, plane(v3(0, 0, z0), vector(0, 0, 1), vector(1, 0, 0)), pts, z1 - z0);
}

// same, with every corner rounded to radius r
function prismR(context is Context, id is Id, pts is array, r is number, z0 is number, z1 is number)
{
    prismPlaneR(context, id, plane(v3(0, 0, z0), vector(0, 0, 1), vector(1, 0, 0)), pts, r, z1 - z0);
}

// closed polygon in a plane's own (x, y) coordinates, extruded along its normal
function prismPlane(context is Context, id is Id, pl is Plane, pts is array, depth is number)
{
    prismPlaneR(context, id, pl, pts, 0, depth);
}

// rounded version: r applies to every corner unless a point carries its own
// radius as a third value ([x, y, r]); radii shrink where an edge is too short
function prismPlaneR(context is Context, id is Id, pl is Plane, pts is array, r is number, depth is number)
{
    const skId = id + "sk";
    const sk = newSketchOnPlane(context, skId, { "sketchPlane" : pl });
    sketchRounded(sk, pts, r);
    skSolve(sk);
    opExtrude(context, id + "ex", {
                "entities" : qSketchRegion(skId),
                "direction" : pl.normal,
                "endBound" : BoundingType.BLIND,
                "endDepth" : depth * MM });
    opDeleteBodies(context, id + "del", { "entities" : qCreatedBy(skId, EntityType.BODY) });
}

function sketchRounded(sk is Sketch, pts is array, r is number)
{
    const n = size(pts);
    var t1 = [];
    var t2 = [];
    var mid = [];
    var has = [];
    for (var i = 0; i < n; i += 1)
    {
        const ip = (i + n - 1) % n;
        const inx = (i + 1) % n;
        const p = vector(pts[i][0], pts[i][1]);
        const pp = vector(pts[ip][0], pts[ip][1]);
        const pn = vector(pts[inx][0], pts[inx][1]);
        const ri = size(pts[i]) > 2 ? pts[i][2] : r;
        const ua = normalize(pp - p);
        const ub = normalize(pn - p);
        const phi = acos(max(-1, min(1, dot(ua, ub))));
        var t = 0;
        var rr = ri;
        if (ri > 0 && phi < 178 * degree && phi > 2 * degree)
        {
            t = ri / tan(phi / 2);
            const tMax = 0.49 * min(norm(pp - p), norm(pn - p));
            if (t > tMax)
            {
                t = tMax;
                rr = t * tan(phi / 2);
            }
        }
        if (t > 0.005)
        {
            const m = normalize(ua + ub);
            const ctr = p + m * (rr / sin(phi / 2));
            t1 = append(t1, p + ua * t);
            t2 = append(t2, p + ub * t);
            mid = append(mid, ctr - m * rr);
            has = append(has, true);
        }
        else
        {
            t1 = append(t1, p);
            t2 = append(t2, p);
            mid = append(mid, p);
            has = append(has, false);
        }
    }
    for (var i = 0; i < n; i += 1)
    {
        const j = (i + 1) % n;
        if (norm(t1[j] - t2[i]) > 0.005)
            skLineSegment(sk, "l" ~ i, { "start" : t2[i] * MM, "end" : t1[j] * MM });
        if (has[i])
            skArc(sk, "a" ~ i, { "start" : t1[i] * MM, "mid" : mid[i] * MM, "end" : t2[i] * MM });
    }
}

// text cut 0.4 mm into the z = 0 face, readable from above
function engraveText(context is Context, id is Id, txt is string, cx is number, cy is number, w is number, h is number)
{
    const skId = id + "sk";
    const sk = newSketchOnPlane(context, skId, { "sketchPlane" : plane(v3(0, 0, -0.4), vector(0, 0, 1), vector(1, 0, 0)) });
    skText(sk, "txt", { "text" : txt, "fontName" : "OpenSans-Bold.ttf",
                "firstCorner" : vector(cx - w / 2, cy - h / 2) * MM, "secondCorner" : vector(cx + w / 2, cy + h / 2) * MM });
    skSolve(sk);
    opExtrude(context, id + "ex", {
                "entities" : qSketchRegion(skId, true),
                "direction" : vector(0, 0, 1),
                "endBound" : BoundingType.BLIND,
                "endDepth" : 1.0 * MM });
    opDeleteBodies(context, id + "del", { "entities" : qCreatedBy(skId, EntityType.BODY) });
}

// union every solid recorded under addKind, then subtract everything under cutKindName
function finishPart(context is Context, reg is box, addKind is string, cutKindName is string) returns Query
{
    const adds = qKind(reg, addKind);
    if (size(evaluateQuery(context, adds)) > 1)
        opBoolean(context, nid(reg, "ops"), { "tools" : adds, "operationType" : BooleanOperationType.UNION });
    cutKind(context, reg, adds, cutKindName);
    return adds;
}

function cutKind(context is Context, reg is box, target is Query, kind is string)
{
    const cuts = qKind(reg, kind);
    if (size(evaluateQuery(context, cuts)) > 0)
        opBoolean(context, nid(reg, "ops"), { "tools" : cuts, "targets" : target, "operationType" : BooleanOperationType.SUBTRACTION });
}

// Fillet every convex edge of a part. Edges are grouped into tangent-connected
// chains (a chain must be rounded in one go or not at all). All chains are tried
// at once; if the kernel refuses, the set is split in half recursively, so one
// awkward chain never stops the rest from being rounded. Edges are tracked by
// their midpoints because fillets renumber the topology.
function roundConvexEdges(context is Context, reg is box, part is Query, r is number) returns string
{
    if (r <= 0)
        return "off";
    const edges = evaluateQuery(context, qOwnedByBody(part, EntityType.EDGE));
    var isConvex = {};
    for (var e in edges)
    {
        var convex = false;
        try silent
        {
            convex = evEdgeConvexity(context, { "edge" : e }) == EdgeConvexityType.CONVEX;
        }
        if (convex)
            isConvex[transientQueriesToStrings(e)] = true;
    }
    var used = {};
    var chains = [];
    var total = 0;
    for (var e in edges)
    {
        const k = transientQueriesToStrings(e);
        if (isConvex[k] != true || used[k] == true)
            continue;
        var mids = [];
        for (var m in evaluateQuery(context, qUnion([e, qTangentConnectedEdges(e)])))
        {
            const km = transientQueriesToStrings(m);
            if (used[km] == true || isConvex[km] != true)
                continue;
            used[km] = true;
            mids = append(mids, evEdgeTangentLine(context, { "edge" : m, "parameter" : 0.5 }).origin);
        }
        if (size(mids) > 0)
        {
            chains = append(chains, mids);
            total += size(mids);
        }
    }
    if (total == 0)
        return "no edges";
    const failed = new box([]);
    filletSplit(context, reg, part, chains, r, failed);
    // chains that refuse the full radius get a smaller one rather than staying sharp
    var smaller = 0;
    for (var f in [0.6, 0.35])
    {
        var still = [];
        for (var c in failed[])
        {
            if (tryFillet(context, reg, part, c, r * f))
                smaller += size(c);
            else
                still = append(still, c);
        }
        failed[] = still;
    }
    var nf = 0;
    for (var c in failed[])
        nf += size(c);
    if (nf == 0)
        return "all " ~ total ~ " edges" ~ (smaller > 0 ? " (" ~ smaller ~ " at a smaller radius)" : "");
    var msg = (total - nf) ~ " of " ~ total ~ " edges; sharp chains at";
    for (var k = 0; k < min(size(failed[]), 10); k += 1)
    {
        const m = failed[][k][0];
        msg = msg ~ " (" ~ fmt1(m[0] / MM) ~ "," ~ fmt1(m[1] / MM) ~ "," ~ fmt1(m[2] / MM) ~ ")x" ~ size(failed[][k]);
    }
    return msg;
}

function filletSplit(context is Context, reg is box, part is Query, chains is array, r is number, failed is box)
{
    if (size(chains) == 0)
        return;
    var mids = [];
    for (var c in chains)
        mids = concatenateArrays([mids, c]);
    if (tryFillet(context, reg, part, mids, r))
        return;
    if (size(chains) == 1)
    {
        // only count it if some of the chain still exists (a neighbour's fillet may have consumed it)
        var alive = 0;
        for (var m in chains[0])
            alive += size(evaluateQuery(context, qContainsPoint(qOwnedByBody(part, EntityType.EDGE), m)));
        if (alive > 0)
            failed[] = append(failed[], chains[0]);
        return;
    }
    const h = floor(size(chains) / 2);
    filletSplit(context, reg, part, subArray(chains, 0, h), r, failed);
    filletSplit(context, reg, part, subArray(chains, h, size(chains)), r, failed);
}

function tryFillet(context is Context, reg is box, part is Query, mids is array, r is number) returns boolean
{
    var qs = [];
    for (var m in mids)
        qs = append(qs, qContainsPoint(qOwnedByBody(part, EntityType.EDGE), m));
    const q = qUnion(qs);
    if (size(evaluateQuery(context, q)) == 0)
        return true;
    var ok = false;
    try silent
    {
        opFillet(context, nid(reg, "ops"), { "entities" : q, "radius" : r * MM });
        ok = true;
    }
    return ok;
}

function massOf(context is Context, q is Query, densityGcc is number) returns number
{
    return evVolume(context, { "entities" : q }) / (centimeter ^ 3) * densityGcc;
}

// a fit-check part built from a list of shapes, carrying a real-world mass
function dummy(context is Context, reg is box, kind is string, specs is array, name is string, col is Color, massG is number) returns array
{
    for (var sp in specs)
    {
        if (sp[0] == "B")
            box3(context, nid(reg, kind), sp[1], sp[2]);
        else
            zcyl(context, nid(reg, kind), sp[1], sp[2], sp[3], sp[4], sp[5]);
    }
    const q = finishPart(context, reg, kind, kind ~ "-nocut");
    const vol = evVolume(context, { "entities" : q });
    setProperty(context, { "entities" : q, "propertyType" : PropertyType.NAME, "value" : name });
    setProperty(context, { "entities" : q, "propertyType" : PropertyType.APPEARANCE, "value" : col });
    setProperty(context, { "entities" : q, "propertyType" : PropertyType.MATERIAL, "value" : material("Fit dummy", massG * gram / vol) });
    return [q, massG];
}

// one decimal place, as a string
function fmt1(x is number) returns string
{
    const r = round(abs(x) * 10);
    const sign = (x < 0 && r > 0) ? "-" : "";
    return sign ~ floor(r / 10) ~ "." ~ (r - 10 * floor(r / 10));
}
