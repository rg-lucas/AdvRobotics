function [p_prime, H] = TwoDof_KinematicModels(theta)
  %% initial robot configuration
  %% unit length link's points' positions [ base, tip]
  joint1 = zeros (2,1); joint2 = [1;0]; tip = [1;0.5];
  Link1 = [ joint1, joint2 ]; lenLink1 = hypot(joint2(1),joint2(2));% We create the first link
  Link2 = [ joint2, tip]; lenLink2 = hypot((tip(1)-joint2(1)),(tip(2)-joint2(2)));% We create the second link

  p1 = expm( skew2D(deg2rad(theta(1))) )*[lenLink1;0];
  p2 = p1 + expm( skew2D(deg2rad(theta(1) + theta(2))) )*[lenLink2;0];

  H = p2;
  p_prime = [p1;p2];

end

