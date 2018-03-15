function [ ErrorVector, Jacobian_Node ] = ConstantScalar( Nodes_array , Measurement_values )
X1=Nodes_array{1};
X2=Nodes_array{2};

ErrorVector= X1 - X2;

Jacobian_Node{1}  = eye(size(X1));
Jacobian_Node{2} = -eye(size(X2));
end