function Param2Sxn(self,Parameters,loc)
%Reads RAMPS parameters file and populates section object
% loc - interior or exterior girder
self.nSpans = Parameters.Spans; % number of spans
self.L  = Parameters.Length; % span length [in]
self.Lb = 0; % unbraced length
self.Es = Parameters.Beam.E; % young's modulus, steel [psi]
self.Fy = Parameters.Beam.Fy; % yield strength of flanges and web [psi]
self.fc = Parameters.Deck.fc; % compressive strength of concrete deck
self.ts = Parameters.Deck.t; % thickness of concrete deck [in]
self.dh = 0; %Parameters. % depth of haunch [in]
switch loc
    case 'Int'
        self.be = Parameters.Deck.be.beInt; % effective width of concrete deck [in]
        self.tw = Parameters.Beam.Int.tw; % thickness of web [in]
        self.dw = Parameters.Beam.Int.ind; % depth of web
        self.bf_top = Parameters.Beam.Int.bf; % width of top flange [in]  
        self.tf_top = Parameters.Beam.Int.tf; % thickness of top flange [in]
        self.bf_bot = Parameters.Beam.Int.bf;% width of bottom flange [in]
        self.tf_bot = Parameters.Beam.Int.tf;% thickness of bottom flange [in]
        self.wDL = Parameters.Demands.Int.SL.DeadLoad.wDL;
        self.wSDL = Parameters.Demands.Int.SL.DeadLoad.wSDL;
        self.wSDW = Parameters.Demands.Int.SL.DeadLoad.wSDW;
    case 'Ext'
        self.be = Parameters.Deck.be.beExt; % effective width of concrete deck [in]
        self.tw = Parameters.Beam.Ext.tw; % thickness of web [in]
        self.dw = Parameters.Beam.Ext.ind; % depth of web
        self.bf_top = Parameters.Beam.Ext.bf; % width of top flange [in]  
        self.tf_top = Parameters.Beam.Ext.tf; % thickness of top flange [in]
        self.bf_bot = Parameters.Beam.Ext.bf;% width of bottom flange [in]
        self.tf_bot = Parameters.Beam.Ext.tf;% thickness of bottom flange [in]
        self.wDL = Parameters.Demands.Ext.SL.DeadLoad.wDL;
        self.wSDL = Parameters.Demands.Ext.SL.DeadLoad.wSDL;
        self.wSDW = Parameters.Demands.Ext.SL.DeadLoad.wSDW;
end
        

end

