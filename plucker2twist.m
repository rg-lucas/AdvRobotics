% unit twist from a plucker line and pitch
function xi = plucker2twist(L,h)
  %L: Euclidean normalized Plucker line
  %h: pitch of the twist (ratio of translation to rotation)
  %xi: unit twist

  u = L(1:3); %unit direction vector of the line
  m = L(4:6); %moment of the line

  xi = [u;m] + h*[zeros(3,1);u];

end
