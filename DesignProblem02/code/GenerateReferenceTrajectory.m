function q2 = GenerateReferenceTrajectory(t)
    if t < 6
        q2 = 0
    else
        %Fun Equation
        q2 = 0.2*sin(0.1*t-0.6)*cos(0.01*pi*t^1.6-0.6)
        %Other Equation
        %q2 = sin(t)/10
    end
end