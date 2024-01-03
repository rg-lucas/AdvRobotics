function UnitDQ = normalizeDQ( ninUnitDQ)
  XR_bar = nonUnitDQ(1:4);
  XD_bar = nonUnitDQ(5:8);

  YR_bar = XR_bar / norm(XR_bar);
  YD_bar = XD_bar / norm(XR_bar);

  sv = qprod( YD_bar, quartenionConjugateforQ( YR_bar) );
  v = sv(2:4);

  UnitDQ = YR_bar; qprod( [0;v], YR_bar) ];

end

