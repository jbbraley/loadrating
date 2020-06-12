function Parameters = AASHTODesign(Parameters, Options)
%% Get Initial Design Configuration
if strcmp(Parameters.Design.Code, 'ASD')
    % Lanes
    if Parameters.RoadWidth >= 20*12 && Parameters.RoadWidth <= 24*12
        Parameters.NumLane = 2;
    elseif Parameters.RoadWidth > 24*12
        Parameters.NumLane = floor(Parameters.RoadWidth/144);
    else
        Parameters.NumLane = 1;
    end
    
    % Live Load Deflection
    Parameters.Design.maxDelta = 800;
    
    % Impact
    Parameters.Design.Im = 50./(Parameters.Length/12+125);
    if Parameters.Design.Im > 0.3
        Parameters.Design.Im = 0.3;
    end
        
    % Determine minimum stiffness for gamma and span length
    % LL Reduction Factor
    if Parameters.NumLane <= 2
        Parameters.Design.MultiPres = 1;
    elseif Parameters.NumLane == 3
        Parameters.Design.MultiPres = 0.9;
    else
        Parameters.Design.MultiPres = 0.75;
    end
    
    % Modulous Ratio
    Parameters.Deck.N = Parameters.Beam.E/Parameters.Deck.E;
%     if Parameters.Deck.fc>=2400 && Parameters.Deck.fc<2900
%         Parameters.Deck.N = 10;
%     elseif Parameters.Deck.fc>=2900 && Parameters.Deck.fc<3600
%         Parameters.Deck.N = 9;
%     elseif Parameters.Deck.fc>=3600 && Parameters.Deck.fc<4600
%         Parameters.Deck.N = 8;
%     elseif Parameters.Deck.fc>=4600 && Parameters.Deck.fc<6000
%         Parameters.Deck.N = 7;
%     else
%         Parameters.Deck.N = 6;
%     end
%     
    % Distribution Factors - Axle Distribution is 1/2 of wheel load
    % distribution
    Parameters.Design.DF = Parameters.GirderSpacing/5.5/12/2;   
    
elseif strcmp(Parameters.Design.Code, 'LFD')
    Parameters.Design.Code = 'LFD';
    
elseif strcmp(Parameters.Design.Code, 'LRFD')
    % Diaphragms ----------------------------------------------------------
    if nargin<2 || Options.Dia.autoConfig
        if Parameters.SkewNear > 20 || Parameters.SkewFar > 20
            Parameters.Dia.Config = 'Normal';
        else
            Parameters.Dia.Config = 'Parallel';
        end
    end
    
    % slenderness ratio
    if Parameters.SkewNear > 20 || Parameters.SkewFar > 20
        Parameters.Dia.slendernessR = 140;
    else
        Parameters.Dia.slendernessR = 120;
    end
    Parameters.Dia.K = 1;
%  --------------------------------------------------------------------------   
    % Lanes 
    if Parameters.RoadWidth >= 20*12 && Parameters.RoadWidth <= 24*12
        Parameters.NumLane = 2;
    elseif Parameters.RoadWidth > 24*12
        Parameters.NumLane = floor(Parameters.RoadWidth/144);
    else
        Parameters.NumLane = 1;
    end
    % Live Load Deflection
    Parameters.Design.maxDelta = 800;
    
    % Impact -- Replaced by dynamic load below
    Parameters.Design.Im = 0;
    
    %Dynamic Load Allowance
    Parameters.Design.IM = 1.33;  %To be applied to static wheel load, not pedestrian or lane load
    
    % FAtigue
    Parameters.Design.IMF = 1.15; % Fatigue impact factor
    Parameters.Design.FTH = 12000;
    
    % Modulous Ratio (N)
    Parameters.Deck.N = Parameters.Beam.E/Parameters.Deck.E;
    
    %Multipresence factors - To be used only with Lever Rule
    Parameters.Design.MultiPres = 1;
    Parameters.Design.MulPres(1) = 1.2;
    Parameters.Design.MulPres(2) = 1;
    Parameters.Design.MulPres(3) = 0.85;
    Parameters.Design.MulPres(4) = 0.65;

    % Determine Effective Width of Deck for Interior or Exterior Beam 
    Parameters.Deck.be.beInt = Parameters.GirderSpacing;
    Parameters.Deck.be.beExt = Parameters.Overhang + Parameters.GirderSpacing/2;
   
    % Transform section to short-term/long-term
    % Determine short-term and long-term effective width of deck for
    % interior girder and exterior girder
    Parameters.Deck.be.beInt_st = Parameters.Deck.be.beInt/Parameters.Deck.N;
    Parameters.Deck.be.beInt_lt = Parameters.Deck.be.beInt/(3*Parameters.Deck.N);
    Parameters.Deck.be.beExt_st = Parameters.Deck.be.beExt/Parameters.Deck.N;
    Parameters.Deck.be.beExt_lt = Parameters.Deck.be.beExt/(3*Parameters.Deck.N);

    % Determine short-term and long-term effective area of deck for
    % interior  girder and exterior girder
    Parameters.Deck.A.AInt_st = Parameters.Deck.t*Parameters.Deck.be.beInt_st;
    Parameters.Deck.A.AInt_lt = Parameters.Deck.t*Parameters.Deck.be.beInt_lt;
    Parameters.Deck.A.AExt_st = Parameters.Deck.t*Parameters.Deck.be.beExt_st;
    Parameters.Deck.A.AExt_lt = Parameters.Deck.t*Parameters.Deck.be.beExt_lt;  
    
else
    Parameters.Design.Code = 'NA';
end

% Truck Loads
Parameters.Design = GetTruckLoads(Parameters.Design);

if Parameters.Spans == 1
    Parameters.Beam.CoverPlate.Ratio = 0;
end

end % CoverPlateDesign()