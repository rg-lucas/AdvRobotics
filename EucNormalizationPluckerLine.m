function eL = EucNormalizationPluckerLine( L )

Lh = L(1:3);
eL = L / norm(Lh);

end