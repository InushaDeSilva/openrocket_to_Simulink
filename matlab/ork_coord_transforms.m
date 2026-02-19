function T = ork_coord_transforms()
%ORK_COORD_TRANSFORMS  Coordinate transform utilities for OR↔Aerospace Blockset.
%
%   T = ORK_COORD_TRANSFORMS() returns a struct of function handles:
%
%   Functions:
%     T.or_to_aero_body(v)       - Transform a 3×1 body-frame vector from
%                                  OpenRocket (x-aft, y-right, z-up) to
%                                  Aerospace Blockset (x-fwd, y-right, z-down).
%                                  Equivalent to diag([-1, 1, -1]) * v.
%
%     T.aero_to_or_body(v)       - Inverse of or_to_aero_body (same matrix,
%                                  since diag([-1,1,-1])^2 = I).
%
%     T.transform_inertia(I_or)  - Transform a 3×3 inertia tensor from OR
%                                  body frame to Aerospace Blockset body frame.
%                                  For diagonal tensors this is identity (Ixx,
%                                  Iyy, Izz are frame-invariant for axis-aligned
%                                  transforms with sign flips only).
%                                  For general tensors: R * I_or * R'.
%
%     T.or_cg_to_aero(cg_x_from_nose, rocket_length)
%                                - Convert OR CG (x from nose, positive aft)
%                                  to Aerospace Blockset CG (x from nose,
%                                  positive forward). Returns the signed
%                                  distance from nose in the x-fwd frame.
%
%     T.or_forces_to_aero(CD, CN, q, Sref)
%                                - Convert OR aero coefficients to body-frame
%                                  force components in Aerospace Blockset
%                                  convention. Returns [Fx; Fy; Fz] in the
%                                  x-fwd, y-right, z-down frame.
%
%     T.rotation_matrix()        - Returns the 3×3 rotation matrix R that
%                                  maps OR body frame to Aerospace Blockset
%                                  body frame: diag([-1, 1, -1]).
%
%   Coordinate Frame Definitions:
%
%     OpenRocket Body Frame:
%       x: positive AFT (from nose toward tail)
%       y: positive RIGHT (looking forward)
%       z: positive UP
%       Origin: nose tip
%
%     Aerospace Blockset Body Frame (Custom Variable Mass 6DOF):
%       x: positive FORWARD (toward nose)
%       y: positive RIGHT (starboard)
%       z: positive DOWN
%       Origin: CG (moves with time)
%
%     OpenRocket World Frame:
%       z-up (ENU-like)
%
%     Aerospace Blockset World Frame:
%       NED (North-East-Down)
%
%   Example:
%     T = ork_coord_transforms();
%     R = T.rotation_matrix();
%     % Transform a force vector
%     F_or = [10; 0; 5];   % 10N aft, 5N up in OR frame
%     F_aero = T.or_to_aero_body(F_or);   % [-10; 0; -5] in aero frame
%
%   See also: ork_import, ork_setup_plant

    % The rotation matrix: OR body -> Aerospace Blockset body
    R = diag([-1, 1, -1]);

    T = struct();

    T.rotation_matrix = @() R;

    T.or_to_aero_body = @(v) R * v;

    T.aero_to_or_body = @(v) R * v;  % R is its own inverse

    T.transform_inertia = @(I_or) R * I_or * R';

    T.or_cg_to_aero = @(cg_x_from_nose, rocket_length) ...
        -(cg_x_from_nose);  % In aero frame, nose is at +x, so CG at -cg_x

    T.or_forces_to_aero = @or_forces_to_aero_impl;
end

function F_aero = or_forces_to_aero_impl(CD, CN, q, Sref)
%OR_FORCES_TO_AERO_IMPL  Convert aero coefficients to Aerospace body forces.
%
%   In OR body frame (x-aft):
%     Fx_or = -CD * q * Sref  (drag opposes motion, so negative in x-aft)
%     Fz_or = CN * q * Sref   (normal force, positive up)
%
%   In Aerospace body frame (x-fwd, z-down):
%     Fx_aero = CD * q * Sref   (drag is negative x-fwd, but in OR it was
%               already negative x-aft -> positive x-fwd after sign flip)
%     Wait — let's be precise:
%
%   Actually, OR defines:
%     CD: total drag coefficient (always positive)
%     CN: normal force coefficient (positive = force "up" in OR z-up frame)
%
%   Drag force in OR body frame: along -x direction (opposing forward motion,
%   but x is aft, so drag is along -x_aft = +x_fwd). WAIT. Let me think again.
%
%   In OR: rocket moves in -x direction (x is aft, nose is at x=0).
%   Drag opposes motion, so drag is in +x direction (aft).
%   Fx_or = +CD * q * Sref  (in the +x_aft direction)
%
%   Transform to aero (x-fwd):
%   Fx_aero = -Fx_or = -CD * q * Sref  (drag opposes forward motion)
%
%   Normal force in OR: Fz_or = +CN * q * Sref (upward)
%   Transform to aero (z-down): Fz_aero = -Fz_or = -CN * q * Sref
%
%   So in Aerospace body frame:
%     Fx = -CD * q * Sref   (axial drag, opposing forward motion)
%     Fy = 0                (symmetric rocket)
%     Fz = -CN * q * Sref   (nose-down restoring force)

    F_aero = [-CD * q * Sref; 0; -CN * q * Sref];
end
