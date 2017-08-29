function p = RigidPosition(g)
if size(g,1) ~= 4 || size(g,2) ~= 4
    error('Input must be a valid rigid body transformation')
end
if ~isZero(g(4,1:3)) || g(4,4) ~= 1
    error('Input must be a valid rigid body transformation')
end
p = g(1:3, 4);
