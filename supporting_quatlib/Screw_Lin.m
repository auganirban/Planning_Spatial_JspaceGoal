function [G, Res]= Screw_Lin(P, Q, Tau)
    p_conj = DQConjugate(P);
    prod = DualQMult(p_conj, Q);
    r = powDQ(prod, Tau);
    Res  = DualQMult(P, r);
    G = DQ2Mat(Res);
end
