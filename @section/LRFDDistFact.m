%% LRFDDistFact: Calculates distribution factors for interior and exterior girder for one or two design lanes laoded.

function [DF,DFV] = LRFDDistFact(s, Designation, NumGirder, GirderSpacing, Overhang, Dist2Lane, skew) %% Distribution Factors
% [DFint, DFext, DFVint, DFVext] = LRFDDistFact(Parameters, Parameters.Beam.Int);
% s - section object
% Designation - 'Int' or 'Ext' girder
% Overhang - distance from exterior girder cl to edge of deck in inches
% Dist2Lane - Distance from edge of deck to nearest edge of traffic lane (in)
% skew - bridge skew in degrees

% Assign Span Length (Length of span or average length of both spans for 2-span continuous)
SpanLength = (sum(s.L)/s.nSpans)/12; % [ft]

% Lever Rule
DFLever = LeverRule( GirderSpacing, Overhang, Dist2Lane);

% Girder Spacing [ft]
S = GirderSpacing/12;

% Slab Thickness [in]
ts = s.ts;

%% Distribution Factors calculated using interior cross-section dimensions

% Longitudinal Stiffness Parameter

% Constant Cross Section
eg = s.d/2+ts/2; %distance between NA of beam and NA of deck
Kg = s.Es/s.Ec*(s.Ix+(s.A*(eg)^2)); % Longitudinal stiffness parameter [in^5]



% Distribution of Live Loads for Moment in Interior Beams [4.6.2.2.2]
DFint1 = 0.06 + (S/14)^(0.4)*(S/SpanLength)^(0.3).*(Kg/(12*ts^3*SpanLength))^(0.1);
DFint2 = 0.075 + (S/9.5)^(0.6)*(S/SpanLength)^(0.2)*(Kg/(12*ts^3*SpanLength))^(0.1);

if S < 3.5 || S > 16 || ts < 4.5 || ts > 16 || Kg < 10000 || Kg > 7e+06 % 4.6.2.2.2
    DFint = DFLever; % One design lane loaded
    DFint(2) = DFint; % Two design lanes loaded
else
    if NumGirder == 3
        DFint = min(DFint1, DFint2, DFLever);
        DFint(2) = DFint;
    else
        DFint = DFint1; % One design lane loaded
        DFint(2) = DFint2; % Two design lanes loaded
    end
end

%Distribution of Live Loads per Lane for Moment in Exterior Longitudinal Beams

% Distance from exterior web of exterior beam and the interior edge of curb or traffic barrier
de = (Overhang-Dist2Lane)/12;
e = 0.77 + de/9.1;

if NumGirder == 3
    DFext = min(DFint*e,DFLever);
    DFext(2) = DFext;
else 
    DFext = DFLever; % One design lane loaded
    DFext(2) = e*DFint(2); % two design lanes loaded
end

% Reduction of Load Distribution Factors for skewed supports
if abs(skew)<=10
    theta = abs(min(skew));
    theta_temp = theta;
    if theta>60
        theta_temp = 60;
    end
    if theta<30
        c1 = 0;
    else
        c1 = 0.25*(Kg/(12*SpanLength*ts^3)).^(0.25).*(S/SpanLength).^(0.5);
    end
    DFskewRed = max(1-c1*(tan(theta_temp*pi/180))^(1.5));
else
    DFskewRed = 1;
end

DFint = DFint.*DFskewRed;
DFext = DFext.*DFskewRed;

% Distribution of Live Load for Shear in Interior Longitudinal Beams
if NumGirder == 3
    DFVint = DFLever;
    DFVint(2) = DFVint;
else
    DFVint = 0.36+S/25;
    DFVint(2) = 0.2+S/12-(S/35)^(2);
end

% Distribution of Live Loads per Lane for Shear in Exterior Longitudinal Beams
DFVext = DFLever;
DFVext(2) = (0.6+de/10)*DFVint(2);

% Correction Factor for support shear at obtuse corner
theta = abs(max(skew));
if theta<=60
    DFVskew = max(1+.2*(12*SpanLength*(ts).^(3)/Kg).^(0.3)*tan(theta*pi/180));
else
    DFVskew = 1;
end

DFVint = DFVskew.*DFVint;
DFVext = DFVskew.*DFVext;

if strcmp(Designation, 'Int')
    DF = DFint;
    DFV = DFVint;
elseif strcmp(Designation, 'Ext')
    DF = DFext;
    DFV = DFVext;
end

end

