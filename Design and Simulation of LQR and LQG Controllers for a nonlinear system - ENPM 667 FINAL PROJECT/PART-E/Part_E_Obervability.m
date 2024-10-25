% Part_E_Observability Assessment

function Part_E_Obervability
    % State-Space Matrices

    m1=100; % in kg
    m2=100; % in kg
    M=1000; % in kg
    l1=20; % in meters
    l2=10; % in meters
    g=9.81; % m/s
    
    A=[0 1 0 0 0 0; 
        0 0 -(m1*g)/M 0 -(m2*g)/M 0;
        0 0 0 1 0 0;
        0 0 -((M+m1)*g)/(M*l1) 0 -(m2*g)/(M*l1) 0;
        0 0 0 0 0 1;
        0 0 -(m1*g)/(M*l2) 0 -(g*(M+m2))/(M*l2) 0];
    B=[0; 1/M; 0; 1/(M*l1); 0; 1/(M*l2)];
    C1 = [1 0 0 0 0 0];% C1 matrix to monitor x(t)
    C2 = [0 0 1 0 0 0; 0 0 0 0 1 0];% C2 matrix to monitor theta1(t), theta2(t) 
    C3 = [1 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 1 0];% C3 matrix to monitor x(t), theta2(t) 
    C4 = [1 0 0 0 0 0; 0 0 1 0 0 0; 0 0 0 0 1 0];% C4 matrix to monitor x(t), theta1(t), theta2(t) 

    n = 6 ; % n rows in the A matrix which is 6 in this case

    % Compute Observability Ranks
    Observ_rank1 = ObservabilityRankCalc(A, C1);
    Observ_rank2 = ObservabilityRankCalc(A, C2);
    Observ_rank3 = ObservabilityRankCalc(A, C3);
    Observ_rank4 = ObservabilityRankCalc(A, C4);

    % Display Results
    fprintf('Rank of observability matrix when monitoring x(t) = %d\n', Observ_rank1);
    fprintf('Rank of observability matrix when monitoring theta1(t), theta2(t) = %d\n', Observ_rank2);
    fprintf('Rank of observability matrix when monitoring x(t) and theta2(t) = %d\n', Observ_rank3);
    fprintf('Rank of observability matrix  when monitoring x(t),theta1(t) and theta2(t) = %d\n', Observ_rank4);
    

    % Check and Display Observability
    Observ_check(Observ_rank1, n, 'For case 1 x(t)');
    Observ_check(Observ_rank2, n, 'For case 2 theta1(t), theta2(t)');
    Observ_check(Observ_rank3, n, 'For case 3 x(t), theta2(t)');
    Observ_check(Observ_rank4, n, 'For case 4 x(t), theta1(t), theta2(t)');
end


function Observ_Rank = ObservabilityRankCalc(A, C)
    % Computes the rank of the observability matrix for a given system 
    Observ_Rank = rank([C' A'*C' A'^2*C' A'^3*C' A'^4*C' A'^5*C']);
end

function Observ_check(check_obs, n, result)
    % Checks and prints if a system is observable
    if check_obs == n
        fprintf('%s is observable.\n', result);
    else
        fprintf('%s is not observable.\n', result);
    end
end