clear all; close all; clc

n = 10000;
tempX = zeros(n,1);
tempY = zeros(n,1);

x = randn(n,1);
y = randn(n,1);

%%
% % THIS ONE IS MUCH FASTER (0.000187 seconds)
% tic
% for i=1:n
%     if x(i)<0
%         tempX(i) = -1;
%         if y(i)<0
%             tempY(i) = -1;
%         else
%             tempY(i) = 1;
%         end
%     else
%         tempX(i) = 1;
%         if y(i)<0
%             tempY(i) = -1;
%         else
%             tempY(i) = 1;
%         end
%     end
% end
% toc
% 
% % THIS ONE IS MUCH SLOWER (0.001632 seconds)
% tic
% for i=1:n
%     if x(i)<0 && y(i)<0
%         tempX(i) = -1;
%         tempY(i) = -1;
%     elseif x(i)<0 && y(i)>=0
%         tempX(i) = -1;
%         tempY(i) =  1;
%     elseif x(i)>=0 && y(i)<0
%         tempX(i) =  1;
%         tempY(i) = -1;
%     elseif x(i)>=0 && y(i)>=0
%         tempX(i) =  1;
%         tempY(i) =  1;
%     else
%         disp('Something went wrong!')
%         break
%     end
% end
% toc

%%
% % THIS ONE IS SLOWER (0.00052 seconds)
% tic
% for i=1:n
%     if i==n
%         tempX(i) = -1;
%     elseif i==1
%         tempX(i) = -1;
%     else
%         tempX(i) = 1;
%     end
% end
% toc
% 
% % THIS ONE IS FASTER (0.00040 seconds)
% tic
% for i=1:n
%     if i==1
%         tempX(i) = -1;
%     else
%         tempX(i) = 1;
%     end
% end
% toc

%%
% %THIS ONE IS SLOWER (0.000045 seconds)
% tic
% for i=1:n
%     if i<n
%         tempX(i) = 1;
%     else
%         tempX(i) = -1;
%     end
% end
% toc
% 
% %THIS ONE IS FASTER (0.000039 seconds)
% tic
% for i=1:n
%     if i==n
%         tempX(i) = -1;
%     else
%         tempX(i) = 1;
%     end
% end
% toc









