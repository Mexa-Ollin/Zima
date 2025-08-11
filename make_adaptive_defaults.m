function ctl = make_adaptive_defaults()
ctl.mode  = 'adaptive';
ctl.zeta  = 0.8; 
ctl.wn    = 6.0; 
ctl.alpha = 4.0;
ctl.est   = struct('K',0.05,'T',0.3,'r0',0,'u0',0,'tau',0.5);
end