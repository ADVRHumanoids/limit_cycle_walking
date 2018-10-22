%Controller based on Bezier curves for virtual constraints.
h_dq = [ 
2 - 2*a_leg(3)*q(1) - 3*a_leg(4)*q(1)^2 - a_leg(2), 1, 0;
a_waist(2) + 2*a_waist(3)*q(1) + 3*a_waist(4)*q(1)^2, 0, 1;
];

h_dq_dq = [ 
-q_dot(1)*(2*a_leg(3) + 6*a_leg(4)*q(1)), 0, 0, 2 - 2*a_leg(3)*q(1) - 3*a_leg(4)*q(1)^2 - a_leg(2), 1, 0;
q_dot(1)*(2*a_waist(3) + 6*a_waist(4)*q(1)), 0, 0, a_waist(2) + 2*a_waist(3)*q(1) + 3*a_waist(4)*q(1)^2, 0, 1;
];

