
state = 1;
QValues = Q_Table_Creator();
%QValues(255,4) = 10;

% [w_left, w_right] = chooseBestState(QValues, state);
%{
for i=1:1:25
    disp(chooseNextState(0.5, 1, QValues));
end


wheels_val = [-5,-4.6] ;
updateQtble(QValues, state, wheels_val,3);
%}
%searching for the previous action and assign reward to it
%returns 0 for update failed, 1 for success
function u = updateQtble(QValues, state, wheels_val, reward)
    u=0;
    global QValues;
    w_l=wheels_val(1);
    w_r=wheels_val(2);
    QTableLen = length(QValues);
    for i = 1: 1: QTableLen
        if (QValues(i,1)==state) && (QValues(i,2) == w_l) && (QValues(i,3) == w_r)
            QValues(i,4)=reward;
            u = 1;  %update sucessfully
        end
    end
end

%   reward funtion
%   input: s_old, s
%   output: integer value reward
%   bounds: s_old and s are in the range of [1,11]
function r = reward(obs_old , obs) 
    if obs== 12
        r = -20;
    else
        x = int8((obs-1)/2)+1;      % x = current state
        y = int8((obs_old-1)/2)+1;  % y = previous state
        if x == x
           t = (0.5*x*x-7.5*x+27); 
           r= int8(t);
        end
        if x < y
            t = 0.5*(x^2)-7.5*x+20+y;
            r= int8(t);
        end
        if x > y
            t = (-0.5*(x^2)+0.5*x-1+y);
            r= int8(t);
        end
    end
end

function reward = r(obs_old , obs)
    x = int8(obs-1)/2;      % x = previous state
    y = int8(obs_old-1)/2;  % y = current state
    if obs == obs_old
       reward = int8(0.5*(x^2)-6.5*x+20); 
    end
    %if line move from outer sensor to middle, give it extra reward 
    if obs < obs_old
        reward = 10 - x + y;
    end
    %if line move from middle to outer sensor, penalize the algorithm
    if obs > obs_old
        reward = y - x
    end

end

function [w_l, w_r]= chooseNextState(epsilon, state, QValues)
    % choose best QValue given state
    if rand(1, 1) <= epsilon
        wheel_vals = chooseBestState(QValues, state);
        w_l = wheel_vals(1);
        w_r = wheel_vals(2);
    else
    % Choose random QValue given state
        wheel_vals = chooseRandValueForState(QValues, state);
        w_l = wheel_vals(1);
        w_r = wheel_vals(2);
    end

end


function wheels_val = chooseRandValueForState(QValues, state)
    total_states = 11;
    QTableLen = length(QValues);
    QStateSize = QTableLen / total_states;
    
    end_of_possible_loc = (QStateSize * state) + 1;
    start_of_possible_loc = end_of_possible_loc - QStateSize;

    randValueLocForState = int32(rand(1, 1)*QStateSize + start_of_possible_loc);
    wheels_val = QValues(randValueLocForState, 2:3);
end

function wheels_val = chooseBestState(QValues, state)
    % Given a state s
    % search for max value
    total_states = 11;
    QTableLen = length(QValues);
    QStateSize = QTableLen / total_states;
    
    end_of_possible_loc = (QStateSize * state) + 1;
    start_of_possible_loc = end_of_possible_loc - QStateSize;
    
    max = 0;
    bestValueLocForState = 1;
    for i=start_of_possible_loc:1:end_of_possible_loc
        curr_state = QValues(i, 1:4);
        if curr_state(4) > max
            bestValueLocForState = i;
            max = curr_state(4);
        end
    end
    wheels_val = QValues(bestValueLocForState, 2:3);
end

% disp(find(QValues(1:length(QValues)-1) ))

function [w_l, w_r]= getNextAction(epsilon, state)
    % choose best QValue given state
    if rand(1, 1) <= epsilon
        [w_l, w_r] = chooseBestState(QValues, state);
    else
    % Choose random QValue given state
        [w_l, w_r] = chooseRandValueForState(QValues, state);
    end

end

function Q_Table = Q_Table_Creator (~)
    max = 5;
    w_l = max;
    w_r = max;
    steps = 10;
    
    minMax = 2 * max;
    bucketSize = minMax / steps;
    
    w_l_potential = [];
    w_r_potential = [];
    
    bucket_pointer = -1*max;
    
    sensorOptions = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11];
    % Find each possible new state for r_wheel and l_wheel
    for i=1:1:steps+1
        w_l_potential = [w_l_potential; [bucket_pointer]];
        w_r_potential = [w_r_potential; [bucket_pointer]];
        bucket_pointer = bucket_pointer + bucketSize;
    end
    
    l_r_combined_options = [];
    
    for i=1:1:steps+1
        for j=1:1:steps+1
            potential_group = [];
            potential_group = [potential_group; [w_l_potential(i), w_r_potential(j)]];
            l_r_combined_options = [l_r_combined_options; potential_group];
        end
    end
    
    Q_Table = [];
    
    for i=1:1:11
        for k=1:1:11
           for j=1:1:((steps+1)*(steps+1))
                potential_group_with_sensor = [];
                potential_group_with_sensor = [potential_group_with_sensor; i, k, l_r_combined_options(j, :)];
                Q_Table = [Q_Table; [potential_group_with_sensor, 0]];
           end
        end
    end
    
end

