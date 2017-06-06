function qinv = myquatinv(qin)
    quatconj = [ qin(1)  -qin(2:4) ];
    qnorm = norm(qin);
    qinv  = quatconj/(qnorm);
end
