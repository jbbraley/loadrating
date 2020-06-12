function [ DF ] = LeverRule( GirderSpacing, Overhang, Dist2Lane  )
%Distance to load furthest to exterior
FarLoadDist = max(GirderSpacing)+Overhang-Dist2Lane-24;

%Number of loads that will fit before the second girder in
NumLoads = floor(FarLoadDist/(6*12))+1;

% Apply multi-presence factor
lanes = ceil(NumLoads/2);
if lanes>3
    lanes = 4;
elseif lanes < 1
    lanes = 1;
end
% LRFD multipresence factors
MulPres(1) = 1.2;
MulPres(2) = 1;
MulPres(3) = 0.85;
MulPres(4) = 0.65;
MultiPresence = MulPres(lanes);

%Distance to the load closest to second girder in
LastLoadDist = rem(FarLoadDist, 6*12);

%Distance from second girder to all loads
LoadDist = [FarLoadDist:-(6*12):LastLoadDist];

%Distribution factor
DF = (sum(LoadDist)/min(GirderSpacing)*MultiPresence)/2;
end

