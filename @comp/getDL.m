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

function [wDL, wSDL] = GetSectionForces(self, numGirder, girderspacing, dia, sw, bar)
% Parameters.Demands.Int = GetSectionForces(Parameters.Beam.Int, Parameters, 'LRFD', 'Int');

% Find DL and LL responses from Single Line FE Approximation

%%%%%%%%%%%%%%%%% DEAD LOAD %%%%%%%%%%%%%%%%%%

% Deck 
DLdeck = self.be*self.ts*0.086705495066; %((Parameters.Width*Parameters.Deck.t)/Parameters.NumGirder)*Parameters.Deck.density; % lb/in.

% Girder
DLstringer = self.A*(490/12^3)*(1.06); % lb/in. includes 6 percent for connections

% Diaphragms
if strcmp(dia.type, 'beam') 
    DLdiaphragm = dia.A*dia.NumDia*girderspacing*(490/12^3)*1.06./self.L; %lb/in.
else
    DLdiaphragm = dia.A*dia.NumDia*(2*sqrt(girderspacing^2+self.d^2))*(490/12^3)*1.06./self.L; %lb/in.
end

% Superimposed dead loads
DLcurb = (2*(sw.width)*sw.height)*(0.086705495066)/numGirder; %lb/in. per girder
if nargin>5
    DLparapet = bar.width*bar.height*2*(0.086705495066)/numGirder; %lb/in. per girder
else
    DLparapet = 0;
end

% Total Non-Superimposed Dead Load
wDL = max(DLdeck+DLstringer+DLdiaphragm); %lb/in.

% Total Superimosed Dead Load
wSDL = DLcurb + DLparapet;

% % Dead Loads from wearing surface
% wSDW = self.be*Deck.WearingSurface*Deck.density; % lb/in.




end % GetSectionForces()
