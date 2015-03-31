% Plots trajectory given rotation matrices and translation vectors
%   Rs - 3x3xN rotation matrices between each pair of successive frames
%   ts - 3xN translation vectors between each pair of successive frames
function plotRt(Rs, ts)
    N = size(Rs, 3);
    
    Ts = zeros(4, 4, N);
    for i=1:N
        Ts(:, :, i) = inv([Rs(:, :, i), ts(:, i); 0 0 0 1]);
    end
    plotT(Ts);
    
%     Rwi = zeros(3, 3, N+1); % Cumulative rotation matrices wrt world frame
%     Rwi(:, :, 1) = eye(3, 3); % First element is identity
%     tw = zeros(3, N+1); % Cumulative translation vectors wrt world frame
%     figure;
%     xlim([-5, 5]);
%     ylim([-5, 5]);
%     zlim([-5, 5]);
%     xlabel('x');
%     ylabel('y');
%     zlabel('z');
%     hold on;
%     scatter3(tw(1, 1), tw(2, 1), tw(3, 1), 'bo');
%     
%     for i=1:N
%         i
%         if (mod(i, 10) == 0)
%             pause;
%         end
%         Rii1 = Rs(:, :, i);
%         tii1 = ts(:, i);
%         
%         Rwi(:, :, i+1) = Rwi(:, :, i) * Rii1;
%         % Frame transformation from body to world frame, using the fact that
%         % R' = inv(R) for orthogonal/rotation matrices
%         tw(:, i+1) = tw(:, i) + inv(Rwi(:, :, i)) * tii1; %Rwi(:, :, i)' * t;
% %         tw(:, i+1) = Rii1 * tw(:, i) + tii1;
%         
%         scatter3(tw(1, i+1), tw(2, i+1), tw(3, i+1), 'bo');
%         %scatter(twi(1, i+1), twi(2, i+1), 'bo');
%     end
%     hold off;
end