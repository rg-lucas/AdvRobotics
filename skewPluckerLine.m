function eL_skew = skewPluckerLine( eL )

 u = eL(1:3); % unit direction axis
 m = eL(4:6); % moment
 
 eL_skew = [ skew(u),m; zeros(1,4)];

end
