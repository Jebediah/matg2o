function [ ErrorVector, Jacobian_Node ] = LidarPrior( Nodes_array , Measurement_values )
%factor for 1 d position and velocity link

PtA = Nodes_array{1};
PtB = Nodes_array{2};

Pm = Measurement_values;

ErrorVector = [ PtA + PtB - Pm ];

Jacobian_PtA = [ 1 ];
Jacobian_PtB = [ 1 ];

Jacobian_Node{1} = Jacobian_PtA;
Jacobian_Node{2} = Jacobian_PtB;

end