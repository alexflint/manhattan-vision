function [ geom ] = make_geometry( vl, vr )

check isscalar(vl);
check isscalar(vr);

geom = struct('vl', vl, 'vr', vr, 'vs', [vl vr]);

end

