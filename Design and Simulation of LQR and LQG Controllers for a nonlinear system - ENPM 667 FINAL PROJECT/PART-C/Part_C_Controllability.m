% Part_C_Controllability Conditions for Linearized System

function Part_C_Controllability

     % Initializing the matrix variables for matrix A and B
    syms M m1 m2 l1 l2 g;

    % Call the function to check controllability of the system
    Control_check(M, m1, m2, l1, l2, g);
end

function Control_check(M, m1, m2, l1, l2, g)
    % Constructing A and B matrices
    A = [0 1 0 0 0 0; 
         0 0 -(g*m1)/M 0 -(g*m2)/M 0;
         0 0 0 1 0 0;
         0 0 -((M+m1)*g)/(M*l1) 0 -(m2*g)/(M*l1) 0;
         0 0 0 0 0 1;
         0 0 -(m1*g)/(M*l2) 0 -(g*(M+m2))/(M*l2) 0];
    B = [0; 1/M; 0; 1/(M*l1); 0; 1/(M*l2)];

    % the Controllability Matrix
    Ctrl = [B A*B A^2*B A^3*B A^4*B A^5*B];

    % Displaying the properties of the controllability matrix
    disp_func(Ctrl, 'Controllability Matrix');

    % Adjusting for l1 = l2 to check for the condtion of controllability
    Ctrl_new = subs(Ctrl, l1, l2);
    disp_func(Ctrl_new, 'Controllability Matrix for l1 = l2');
end

function disp_func(matrix, title)
    % Displays rank and determinant of a matrix 
    fprintf('\n%s:\n', title);
    disp(matrix);
    fprintf('Determinant: %s\n', char(det(matrix)));
    fprintf('Rank: %d\n', rank(matrix));
    % Check controllability
    if rank(matrix) == size(matrix, 1)
        disp('System is controllable due to rank = n');
    else
        disp('System is not controllable due to rank != n');
    end
end

