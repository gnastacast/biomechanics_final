%% Calculate center of gravity
function xyzCOG = getCOG(xyz, COMtable, jointNames)

xyzCOG = zeros(1,3);
for i = 1:height(COMtable)
    prefixes = "";
    if COMtable{i, 2}
        prefixes = ["r", "l"];
    end
    for prefix = prefixes
        total = zeros(1,3);
        for joint = COMtable{i, 4}{1}'
            jointName = prefix + joint{1};
            jointIDX = jointNames == jointName;
            assert(sum(jointIDX) == 1)
            total = total + xyz(jointIDX,:) .* joint{2};
        end
        xyzCOG = xyzCOG + total .* (COMtable{i, 3} / 100);
    end
end

end