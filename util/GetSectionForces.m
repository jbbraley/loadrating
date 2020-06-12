%% GetSectionForces: Calculates Dead Load and Live Load Responses (Moment, Shear, Extreme Fiber Stresses) based on single line responses to unit loading.
% Assignments:
    % 1. Non-Superimposed Dead Load, DC1 (wDL)
    % 2. Superimposed Dead Load, DC2 (wSDL)
    % 3. Dead Load due to Wearing Surface, DW (wSDW)
    % 4. Dead Load Moment for DC1 (MDL_pos, MDL_neg)
    % 5. Dead Load Moment for DC2 (MSDL_pos, MSDL_neg)
    % 6. Dead Load Moment for DW (MSDW_pos, MSDW_neg)
    % 7. Dead Load Shear for DC1 and DC2 (VDL)
    % 8. Dead Load Shear for DW (VDW)
    % 9. Live Load Moment (MLL_pos, MLL_neg)
    % 10. Live Load Shear (VLL)
    % 11. Live Load Distribution Factors (DF, DFV)

function ArgOut = GetSectionForces(ArgIn, Parameters, Code, Section, fCall)
% Parameters.Demands.Int = GetSectionForces(Parameters.Beam.Int, Parameters, 'LRFD', 'Int');

if isempty(fCall)
    ind = 1;
end

% Find DL and LL responses from Single Line FE Approximation

%%%%%%%%%%%%%%%%% DEAD LOAD %%%%%%%%%%%%%%%%%%

% Deck 
ArgOut.DeadLoad.DLdeck = Parameters.Deck.be.beExt*Parameters.Deck.t*Parameters.Deck.density; %((Parameters.Width*Parameters.Deck.t)/Parameters.NumGirder)*Parameters.Deck.density; % lb/in.

% Girder
if Parameters.Spans == 1
    ArgOut.DeadLoad.DLstringer = ArgIn.A*(490/12^3)*(1.06); % lb/in. includes 6 percent for connections
else
    if ArgIn.CoverPlate.Length > 0
        % Add weight of coverplate if cover plate exists
        ArgOut.DeadLoad.DLstringer = (1-ArgIn.CoverPlate.Ratio)*ArgIn.A*(490/12^3)*(1.06)+(ArgIn.CoverPlate.Ratio)*ArgIn.CoverPlate.A*(490/12^3)*(1.06);
    else
        ArgOut.DeadLoad.DLstringer = ArgIn.A*(490/12^3)*(1.06); % lb/in. includes 6 percent for connections
    end
end

% Concrete Encasement
ArgOut.DeadLoad.DLencasement = 0;
      
% Diaphragms
if strcmp(Parameters.Dia.Type, 'Beam') 
    ArgOut.DeadLoad.DLdiaphragm = Parameters.Dia.A*Parameters.NumDia*Parameters.GirderSpacing*(490/12^3)*1.06./Parameters.Length; %lb/in.
else
    ArgOut.DeadLoad.DLdiaphragm = Parameters.Dia.A*Parameters.NumDia*(2*sqrt(Parameters.GirderSpacing^2+ArgIn.d^2))*(490/12^3)*1.06./Parameters.Length; %lb/in.
end

% Superimposed dead loads
ArgOut.DeadLoad.DLcurb = ((Parameters.Sidewalk.Left+Parameters.Sidewalk.Right)*Parameters.Sidewalk.Height)*(Parameters.Sidewalk.density)/Parameters.NumGirder; %lb/in. per girder
ArgOut.DeadLoad.DLparapet = Parameters.Barrier.Width*Parameters.Barrier.Height*2*Parameters.Barrier.density/Parameters.NumGirder; %lb/in. per girder

% Total Non-Superimposed Dead Load
ArgOut.DeadLoad.wDL = max(ArgOut.DeadLoad.DLdeck+ArgOut.DeadLoad.DLstringer+ArgOut.DeadLoad.DLdiaphragm+ArgOut.DeadLoad.DLencasement); %lb/in.

% Total Superimosed Dead Load
ArgOut.DeadLoad.wSDL = ArgOut.DeadLoad.DLcurb + ArgOut.DeadLoad.DLparapet;

% Dead Loads from wearing surface
ArgOut.DeadLoad.wSDW = Parameters.Deck.be.beInt*Parameters.Deck.WearingSurface*Parameters.Deck.density; % lb/in.

% Dead Load Moment Non-superimposed (Deck, Girder Diaphragms)
ArgOut.DeadLoad.MDL_pos = ArgOut.DeadLoad.wDL.*Parameters.Design.SLG.(Section)(fCall).maxDLM_POI;
ArgOut.DeadLoad.MDL_neg = min(ArgOut.DeadLoad.wDL.*Parameters.Design.SLG.(Section)(fCall).minDLM_POI,[],2);

% Dead Load Moment Superimposed (Sidewalk and Barriers)
ArgOut.DeadLoad.MSDL_pos = ArgOut.DeadLoad.wSDL*Parameters.Design.SLG.(Section)(fCall).maxDLM_POI;
ArgOut.DeadLoad.MSDL_neg = min(ArgOut.DeadLoad.wSDL*Parameters.Design.SLG.(Section)(fCall).minDLM_POI,[],2);

% Dead Load Moment Superimposed (Future wearing surface)
ArgOut.DeadLoad.MSDW_pos = ArgOut.DeadLoad.wSDW*Parameters.Design.SLG.(Section)(fCall).maxDLM_POI;
ArgOut.DeadLoad.MSDW_neg = min(ArgOut.DeadLoad.wSDW*Parameters.Design.SLG.(Section)(fCall).minDLM_POI,[],2);

% Dead Laod Shear (DC1 and DC2)
ArgOut.DeadLoad.VDL = max((ArgOut.DeadLoad.wDL+ArgOut.DeadLoad.wSDL)*Parameters.Design.SLG.(Section)(fCall).maxDLV_POI, [], 2);

% Dead Load Shear (DW)
ArgOut.DeadLoad.VDW = max(ArgOut.DeadLoad.wSDW*Parameters.Design.SLG.(Section)(fCall).maxDLV_POI, [], 2);

%%%%%%%%%%%%%%%%% LIVE LOAD %%%%%%%%%%%%%%%%%%

% Live load moments and shears
ArgOut.LiveLoad.MLL_pos = Parameters.Design.SLG.(Section)(fCall).maxM_POI; %short term composite;
ArgOut.LiveLoad.MLL_neg = min(Parameters.Design.SLG.(Section)(fCall).minM_POI,[],2);
ArgOut.LiveLoad.VLL = max(Parameters.Design.SLG.(Section)(fCall).maxV_POI,[],2);

%%%%%%%%%%%%%%%%% FATIGUE LOAD %%%%%%%%%%%%%%%
[range, loc] = max(abs(Parameters.Design.SLG.(Section)(fCall).minFM - Parameters.Design.SLG.(Section)(fCall).maxFM));
ArgOut.LiveLoad.MFL_max = Parameters.Design.SLG.(Section)(fCall).maxFM(:,loc);
ArgOut.LiveLoad.MFL_min = Parameters.Design.SLG.(Section)(fCall).minFM(:,loc);

% Find factored DL and LL responses for ASD or LRFD

%%%%%%%%%%%%%%%%%%%% ASD %%%%%%%%%%%%%%%%%%%%%

if strcmp(Code, 'ASD')
  % Live Load Using Distribution Factors
    ArgOut.ASD.MLLIm_pos(1,:) = ArgOut.LiveLoad.MLL_pos.*(Parameters.Design.Im+1)*Parameters.Design.DF;
    ArgOut.ASD.MLLIm_pos(2,:) = ArgOut.LiveLoad.MLL_pos.*(Parameters.Design.Im+1)*Parameters.NumLane*Parameters.Design.MultiPres/Parameters.NumGirder;
    ArgOut.ASD.MLLIm_neg(1,:) = ArgOut.LiveLoad.MLL_neg.*(Parameters.Design.Im+1)*Parameters.Design.DF;
    ArgOut.ASD.MLLIm_neg(2,:) = ArgOut.LiveLoad.MLL_neg.*(Parameters.Design.Im+1)*Parameters.NumLane*Parameters.Design.MultiPres/Parameters.NumGirder;

    % Stresses
    if Parameters.Deck.CompositeDesign == 1
        % Strength Limit State 1
        % Positive
        % Top Fibers
        ArgOut.ASD.sigma1 = ArgOut.DeadLoad.MDL_pos/ArgIn.S.STnc + ArgOut.DeadLoad.MSDL_pos/ArgIn.S.STlt + max(ArgOut.ASD.MLLIm_pos,[],1)'/ArgIn.S.STst;
        ArgOut.ASD.fb1 = max(ArgOut.ASD.sigma1);
        % Bottom Fibers
        ArgOut.ASD.sigma2 = ArgOut.DeadLoad.MDL_pos./ArgIn.S.SBnc + ArgOut.DeadLoad.MSDL_pos/ArgIn.S.SBlt + max(ArgOut.ASD.MLLIm_pos,[],1)'/ArgIn.S.SBst;
        ArgOut.ASD.fb2 = max(ArgOut.ASD.sigma2);

        % Strength Limit State 2
        % Positive
        % Top Fibers
        ArgOut.ASD.sigma3 = (ArgOut.DeadLoad.MDL_pos/ArgIn.S.STnc + ArgOut.DeadLoad.MSDL_pos/ArgIn.S.STlt + 2*max(ArgOut.ASD.MLLIm_pos,[],1)'/ArgIn.S.STst)/1.5;
        ArgOut.ASD.fb3 = max(ArgOut.ASD.sigma3);
        % Bottom Fibers
        ArgOut.ASD.sigma4 = (ArgOut.DeadLoad.MDL_pos/ArgIn.S.SBnc + ArgOut.DeadLoad.MSDL_pos/ArgIn.S.SBlt + 2*max(ArgOut.ASD.MLLIm_pos,[],1)'/ArgIn.S.SBst)/1.5;
        ArgOut.ASD.fb4 = max(ArgOut.ASD.sigma4);

        % Negative
        %Strength Limit State 1
        ArgOut.ASD.sigma5 = abs(ArgOut.DeadLoad.MDL_neg)/ArgIn.S.STnc + abs(ArgOut.DeadLoad.MSDL_neg)/ArgIn.S.STnc + max(abs(ArgOut.ASD.MLLIm_neg))'/ArgIn.S.STnc;
        ArgOut.ASD.fb5 = max(ArgOut.ASD.sigma5);

        %Strength Limit State 2
        ArgOut.ASD.sigma6 = (abs(ArgOut.DeadLoad.MDL_neg)/ArgIn.S.STnc + abs(ArgOut.DeadLoad.MSDL_neg)/ArgIn.S.STnc + 2*max(abs(ArgOut.ASD.MLLIm_neg))'/ArgIn.S.STnc)/1.5;
        ArgOut.ASD.fb6 = max(ArgOut.ASD.sigma6);

    else
         % Strength Limit State 1
        % Positive
        % Top Fibers
        ArgOut.ASD.sigma1 = ArgOut.DeadLoad.MDL_pos/ArgIn.S.STnc + ArgOut.DeadLoad.MSDL_pos/ArgIn.S.STnc + max(ArgOut.ASD.MLLIm_pos,[],1)'/ArgIn.S.STnc;
        ArgOut.ASD.fb1 = max(ArgOut.ASD.sigma1);
        % Bottom Fibers
        ArgOut.ASD.sigma2 = ArgOut.DeadLoad.MDL_pos/ArgIn.S.SBnc + ArgOut.DeadLoad.MSDL_pos/ArgIn.S.SBnc + max(ArgOut.ASD.MLLIm_pos,[],1)'/ArgIn.S.SBnc;
        ArgOut.ASD.fb2 = max(ArgOut.ASD.sigma2);

        % Strength Limit State 2
        % Positive
        % Top Fibers
        ArgOut.ASD.sigma3 = (ArgOut.DeadLoad.MDL_pos/ArgIn.S.STnc + ArgOut.DeadLoad.MSDL_pos/ArgIn.S.STnc + 2*max(ArgOut.ASD.MLLIm_pos,[],1)'/ArgIn.S.STnc)/1.5;
        ArgOut.ASD.fb3 = max(ArgOut.ASD.sigma3);
        % Positive
        ArgOut.ASD.sigma4 = (ArgOut.DeadLoad.MDL_pos/ArgIn.S.SBnc + ArgOut.DeadLoad.MSDL_pos/ArgIn.S.SBnc + 2*max(ArgOut.ASD.MLLIm_pos,[],1)'/ArgIn.S.SBnc)/1.5;
        ArgOut.ASD.fb4 = max(ArgOut.ASD.sigma4);

        % Negative
        %Strength Limit State 1
        ArgOut.ASD.sigma5 = abs(ArgOut.DeadLoad.MDL_neg)/ArgIn.S.STnc + abs(ArgOut.DeadLoad.MSDL_neg)/ArgIn.S.STnc + max(abs(ArgOut.ASD.MLLIm_neg))'/ArgIn.S.STnc;
        ArgOut.ASD.fb5 = max(ArgOut.ASD.sigma5);

        %Strength Limit State 2
        ArgOut.ASD.sigma6 = (abs(ArgOut.DeadLoad.MDL_neg)/ArgIn.S.STnc + abs(ArgOut.DeadLoad.MSDL_neg)/ArgIn.S.STnc + 2*max(abs(ArgOut.ASD.MLLIm_neg))'/ArgIn.S.STnc)/1.5;
        ArgOut.ASD.fb6 = max(ArgOut.ASD.sigma6);
    end

    if Parameters.Design.CoverPlate.Ratio == 0
        ArgOut.ASD.fbcmax = max([ArgOut.ASD.fb1,ArgOut.ASD.fb3, abs(ArgOut.ASD.fb5), abs(ArgOut.ASD.fb6)]);
    else
        ArgOut.ASD.fbcmax = max(ArgOut.ASD.fb1,ArgOut.ASD.fb3);
    end  

%%%%%%%%%%%%%%%%%%%% LRFD %%%%%%%%%%%%%%%%%%%%%    
    
elseif strcmp(Code, 'LRFD')
    
    % Live Load Using Distribution Factors
    ArgOut.LRFD.MLL_pos = ArgOut.LiveLoad.MLL_pos.*max(Parameters.Design.DF.(['DF' (Section)]));
    ArgOut.LRFD.MLL_neg = min(ArgOut.LiveLoad.MLL_neg.*max(Parameters.Design.DF.(['DF' (Section)])));
    ArgOut.LRFD.VLL = ArgOut.LiveLoad.VLL.*max(Parameters.Design.DF.(['DFV' (Section)]));

    % Total Factored Moments & Stresses

    % Strength Limit State I
    % Strength I factors: DC = 1.25, DW = 1.5, LL = 1.75
    ArgOut.LRFD.M_pos = 1.25*(ArgOut.DeadLoad.MDL_pos+ArgOut.DeadLoad.MSDL_pos)+1.50*(ArgOut.DeadLoad.MSDW_pos)+1.75*(ArgOut.LRFD.MLL_pos);
    ArgOut.LRFD.M_neg = abs(1.25*(ArgOut.DeadLoad.MDL_neg+ArgOut.DeadLoad.MSDL_neg)+1.50*(ArgOut.DeadLoad.MSDW_neg)+1.75*(ArgOut.LRFD.MLL_neg));
    ArgOut.LRFD.V = 1.25*(ArgOut.DeadLoad.VDL)+1.50*(ArgOut.DeadLoad.VDW)+1.75*(ArgOut.LRFD.VLL);    

    % Positive Region
    ArgOut.LRFD.fbc_pos(1,:) = max(1.25*(abs(ArgOut.DeadLoad.MDL_pos)/ArgIn.S.STnc+abs(ArgOut.DeadLoad.MSDL_pos)/ArgIn.S.STlt)+...
        1.50*(abs(ArgOut.DeadLoad.MSDW_pos)/ArgIn.S.STlt)+...
        1.75*max(abs(ArgOut.LRFD.MLL_pos))/ArgIn.S.STst);
    ArgOut.LRFD.fbt_pos(1,:) = max(1.25*(abs(ArgOut.DeadLoad.MDL_pos)/ArgIn.S.SBnc+abs(ArgOut.DeadLoad.MSDL_pos)/ArgIn.S.SBlt)+...
        1.50*(abs(ArgOut.DeadLoad.MSDW_pos)/ArgIn.S.SBlt)+...
        1.75*max(abs(ArgOut.LRFD.MLL_pos))/ArgIn.S.SBst);

    % Negative Region  
    if Parameters.Spans > 1 
        if ArgIn.CoverPlate.Length > 0
            % Negative Moment Section with cover plate
            ArgOut.LRFD.fbc_neg(1,:) = max(1.25*max((abs(ArgOut.DeadLoad.MDL_neg)/ArgIn.CoverPlate.S.SBnc+abs(ArgOut.DeadLoad.MSDL_neg)/ArgIn.CoverPlate.S.SBnc))+...
                1.50*max(abs(ArgOut.DeadLoad.MSDW_neg))/ArgIn.CoverPlate.S.SBnc+...
                1.75*max(abs(ArgOut.LRFD.MLL_neg))/ArgIn.CoverPlate.S.SBnc);
            ArgOut.LRFD.fbt_neg(1,:) = max(1.25*max((abs(ArgOut.DeadLoad.MDL_neg)/ArgIn.CoverPlate.S.STnc+abs(ArgOut.DeadLoad.MSDL_neg)/ArgIn.CoverPlate.S.STnc))+...
                1.50*max(abs(ArgOut.DeadLoad.MSDW_neg))/ArgIn.CoverPlate.S.STnc+...
                1.75*max(abs(ArgOut.LRFD.MLL_neg))/ArgIn.CoverPlate.S.STnc);
        else
            ArgOut.LRFD.fbc_neg(1,:) = max(1.25*max((abs(ArgOut.DeadLoad.MDL_neg)/ArgIn.S.SBnc+abs(ArgOut.DeadLoad.MSDL_neg)/ArgIn.S.SBnc))+...
                1.50*max(abs(ArgOut.DeadLoad.MSDW_neg))/ArgIn.S.SBnc+...
                1.75*max(abs(ArgOut.LRFD.MLL_neg))/ArgIn.S.SBnc);
            ArgOut.LRFD.fbt_neg(1,:) = max(1.25*max((abs(ArgOut.DeadLoad.MDL_neg)/ArgIn.S.STnc+abs(ArgOut.DeadLoad.MSDL_neg)/ArgIn.S.STnc))+...
                1.50*max(abs(ArgOut.DeadLoad.MSDW_neg))/ArgIn.S.STnc+...
                1.75*max(abs(ArgOut.LRFD.MLL_neg))/ArgIn.S.STnc);
        end
    end

    % Service Limit State II
    % Service II factors: DC = 1.0, DW = 1.0, LL = 1.3

    % Positive Region
    ArgOut.LRFD.fbc_pos(2,:) = max(1*(abs(ArgOut.DeadLoad.MDL_pos)/ArgIn.S.STnc+...%Stress in compression flange
        abs(ArgOut.DeadLoad.MSDL_pos)/ArgIn.S.STlt+abs(ArgOut.DeadLoad.MSDW_pos)/ArgIn.S.STlt)+... 
        1.30*max(abs(ArgOut.LRFD.MLL_pos))/ArgIn.S.STst);
    ArgOut.LRFD.fbt_pos(2,:) = max(1*(abs(ArgOut.DeadLoad.MDL_pos)/ArgIn.S.SBnc+...%Stress in tension flange
        abs(ArgOut.DeadLoad.MSDL_pos)/ArgIn.S.SBlt+abs(ArgOut.DeadLoad.MSDW_pos)/ArgIn.S.SBlt)+... 
        1.30*max(abs(ArgOut.LRFD.MLL_pos))/ArgIn.S.SBst);

    % Negative Region
    if Parameters.Spans > 1 
        if ArgIn.CoverPlate.Length > 0
           ArgOut.LRFD.fbc_neg(2,:) = max(1*max((abs(ArgOut.DeadLoad.MDL_neg)/ArgIn.CoverPlate.S.SBnc+...
                abs(ArgOut.DeadLoad.MSDL_neg)/ArgIn.CoverPlate.S.SBnc)+abs(ArgOut.DeadLoad.MSDW_neg)/ArgIn.CoverPlate.S.SBnc)+...
                1.30*max(abs(ArgOut.LRFD.MLL_neg))/ArgIn.CoverPlate.S.SBnc);    
           ArgOut.LRFD.fbt_neg(2,:) = max(1*max((abs(ArgOut.DeadLoad.MDL_neg)/ArgIn.CoverPlate.S.STnc+...
                abs(ArgOut.DeadLoad.MSDL_neg)/ArgIn.CoverPlate.S.STnc+abs(ArgOut.DeadLoad.MSDW_neg)/ArgIn.CoverPlate.S.STnc))+...
                1.30*max(abs(ArgOut.LRFD.MLL_neg))/ArgIn.CoverPlate.S.STnc);
        else    
           ArgOut.LRFD.fbc_neg(2,:) = max(1*max((abs(ArgOut.DeadLoad.MDL_neg)/ArgIn.S.SBnc+...
                abs(ArgOut.DeadLoad.MSDL_neg)/ArgIn.S.SBnc)+abs(ArgOut.DeadLoad.MSDW_neg)/ArgIn.S.SBnc)+...
                1.30*max(abs(ArgOut.LRFD.MLL_neg))/ArgIn.S.SBnc);    
           ArgOut.LRFD.fbt_neg(2,:) = max(1*max((abs(ArgOut.DeadLoad.MDL_neg)/ArgIn.S.STnc+...
                abs(ArgOut.DeadLoad.MSDL_neg)/ArgIn.S.STnc+abs(ArgOut.DeadLoad.MSDW_neg)/ArgIn.S.STnc))+...
                1.30*max(abs(ArgOut.LRFD.MLL_neg))/ArgIn.S.STnc);
        end
    end

    % Maximum stress in compression flange
    if Parameters.Spans > 1
       ArgOut.LRFD.fbcmax = max(max(ArgOut.LRFD.fbc_pos),max(ArgOut.LRFD.fbc_neg));
    else
       ArgOut.LRFD.fbcmax = max(ArgOut.LRFD.fbc_pos);
    end
    
    % Fatigue I Limit State (factor = 1.50 for infinite life)
    
    % Determine nodes with fixed DOFs in order to determine location of CP
    % region
    SpanLength = round(Parameters.Length/12)'; %[ft]
    Fixed = zeros(Parameters.Spans, 1);
    for ii = 1:Parameters.Spans
        Fixed(ii) = sum(SpanLength(1:ii));
    end
    Fixed = [1;Fixed+1];
    
    if Parameters.Spans > 1
    
        lb = floor(Fixed(2:end-1)-(ArgIn.CoverPlate.Ratio*max(Parameters.Length/12)));
        ub = ceil(Fixed(2:end-1)+(ArgIn.CoverPlate.Ratio*max(Parameters.Length/12))-1);

        if find(loc>=lb & loc <=ub) > 0 % Coverplate region
            ArgOut.LRFD.fT_ftg = 1.5*abs((ArgOut.LiveLoad.MFL_min*(ArgIn.CoverPlate.y.yTnc-ArgIn.CoverPlate.tf)/ArgIn.CoverPlate.I.Ix)-...
                (ArgOut.LiveLoad.MFL_max*(ArgIn.CoverPlate.y.yTnc-ArgIn.CoverPlate.tf)/ArgIn.CoverPlate.I.Ix));
            ArgOut.LRFD.fB_ftg = 1.5*abs((ArgOut.LiveLoad.MFL_min*(ArgIn.CoverPlate.y.yBnc-ArgIn.CoverPlate.tf)/ArgIn.CoverPlate.I.Ix)-...
                (ArgOut.LiveLoad.MFL_max*(ArgIn.CoverPlate.y.yBnc-ArgIn.CoverPlate.tf)/ArgIn.CoverPlate.I.Ix));
        else % positive region
            ArgOut.LRFD.fT_ftg = 1.5*abs((ArgOut.LiveLoad.MFL_min*(ArgIn.y.yTst-ArgIn.tf)/ArgIn.I.Ist)-...
            (ArgOut.LiveLoad.MFL_max*(ArgIn.y.yTst-ArgIn.tf)/ArgIn.I.Ist));
            ArgOut.LRFD.fB_ftg = 1.5*abs((ArgOut.LiveLoad.MFL_min*(ArgIn.y.yBst-ArgIn.tf)/ArgIn.I.Ist)-...
                (ArgOut.LiveLoad.MFL_max*(ArgIn.y.yBst-ArgIn.tf)/ArgIn.I.Ist));
        end  
    else
        ArgOut.LRFD.fT_ftg = 1.5*abs((ArgOut.LiveLoad.MFL_min*(ArgIn.y.yTst-ArgIn.tf)/ArgIn.I.Ist)-...
            (ArgOut.LiveLoad.MFL_max*(ArgIn.y.yTst-ArgIn.tf)/ArgIn.I.Ist));
        ArgOut.LRFD.fB_ftg = 1.5*abs((ArgOut.LiveLoad.MFL_min*(ArgIn.y.yBst-ArgIn.tf)/ArgIn.I.Ist)-...
            (ArgOut.LiveLoad.MFL_max*(ArgIn.y.yBst-ArgIn.tf)/ArgIn.I.Ist));
    end
end

end % GetSectionForces()
