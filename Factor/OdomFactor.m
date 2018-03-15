function [ ErrorVector, Jacobian_Node ] = OdomFactor( Nodes_array , Measurement_values )
%factor for 1 d position and velocity link

PtA = Nodes_array{1};
PtB = Nodes_array{2};
VelA = Nodes_array{3};

dT = Measurement_values;

ErrorVector = [ PtA + dT * VelA - PtB ];

Jacobian_PtA = [ 1 ];
Jacobian_PtB = [ -1];
Jacobian_VelA = [ dT ];

Jacobian_Node{1} = Jacobian_PtA;
Jacobian_Node{2} = Jacobian_PtB;
Jacobian_Node{3} = Jacobian_VelA;

end