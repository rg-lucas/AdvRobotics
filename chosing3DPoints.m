%% create 2 points and check if they are in the range of the robot  or not.

A = 400*(rand(1,3)+1)
B = [400*(rand(1,1)+1), 400*(rand(1,1)+1), 400*(rand(1,1)+1)]


%% Checking if the coordinates x, y aren't outside of the robots reach
%% Using a simplified checking process.

if (A(1) < 854.88 && A(1) > 241.34 && A(2) < 854.88 && A(2) > 241.34 &&
  B(1) < 854.88 && B(1) > 241.34 && B(2) < 854.88 && B(2) > 241.34 &&
  A(3) < 779.88 && A(3) > 166.34 && B(3) < 779.88 && B(3) > 166.34)

  printf('the range is  valid in X, Y, Z\n')

else
  printf('the range is not valid in X,Y, Z\n')

end


