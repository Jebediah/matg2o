function [ ErrorVector, Jacobian_Node ] = PriorVector_Factor( Nodes_array , Measurement_values )
X=Nodes_array{1};
Xm = Measurement_values;

ErrorVector= X - Xm;

Jacobian_Node{1}  = eye(size(X));
end