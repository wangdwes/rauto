% Plots trajectory given homogeneous transformations
%   Ts - 3x3xN rotation matrices between each pair of successive frames
function plotT(Ts)
    N = size(Ts, 3);
    Twi = zeros(4, 4, N+1); % Cumulative homoegeneous transform wrt world frame
    Twi(:, :, 1) = eye(4, 4); % First element is identity
    twi = zeros(4, N+1);
    
    figure;
    xlim([-1, 5]);
    ylim([-3, 3]);
    zlim([-3, 3]);
    xlabel('x/right (m)');
    ylabel('y/down (m)');
    zlabel('z/forward (m)');
    hold on;
    grid on;
    scatter3(twi(1, 1), twi(2, 1), twi(3, 1), 'bo');
    for i=1:N
%         i
%         if (mod(i, 10) == 0)
%             pause;
%         end
        T = Ts(:, :, i);
        
        Twi(:, :, i+1) = Twi(:, :, i) * T;
        % Frame transformation from body to world frame
        twi(:, i+1) = (Twi(:, :, i)) * [0 0 0 1]';
        scatter3(twi(1, i+1), twi(2, i+1), twi(3, i+1), 'bo');
        %scatter(twi(1, i+1), twi(2, i+1), 'bo');
    end
    hold off;
    
    figure;
    subplot(3, 1, 1);
    plot(twi(1, :));
    xlabel('Image Number');
    ylabel('Accumulated x (m)');
    grid on;
    
    subplot(3, 1, 2);
    plot(-twi(2, :));
    xlabel('Image Number');
    ylabel('Accumulated y (m)');
    grid on;
    
    subplot(3, 1, 3);
    plot(twi(3, :));
    xlabel('Image Number');
    ylabel('Accumulated z (m)');
    grid on;
end