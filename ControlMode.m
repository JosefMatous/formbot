classdef ControlMode < int32
    % CONTROLMODE Enumeration class of robot control modes.
    enumeration
        % DEFAULT The "Coupled dynamics formation control" law described in section IV.A
        Default    (0)
        % DAMPING The "Coupled dynamics formation control with passivity-based interrobot damping" law described in section IV.B
        Damping    (1)
        % DEFAULT The "Saturated control" law described in section IV.C
        Saturation (2)
    end
end