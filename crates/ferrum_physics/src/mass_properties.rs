use ferrum_core::math::{Float, Vec3};
use crate::physics_vertex::{Face, Polyhedron};

fn comp_projection_integrals(f: &Face, v: &Vec<Vec3>, A: usize, B: usize) ->
                                                    (Float, Float, Float, Float, Float, Float, Float, Float, Float, Float){
    //Only used here
    let (mut a0, mut a1, mut da): (Float, Float, Float);
    let (mut b0, mut b1, mut db): (Float, Float, Float);
    let (mut a0_2, mut a0_3, mut a0_4, mut b0_2, mut b0_3, mut b0_4): (Float, Float, Float, Float, Float, Float);
    let (mut a1_2, mut a1_3, mut b1_2, mut b1_3): (Float, Float, Float, Float);
    let (mut C1, mut Ca, mut Caa, mut Caaa, mut Cb, mut Cbb, mut Cbbb): (Float, Float, Float, Float, Float, Float, Float);
    let (mut Cab, mut Kab, mut Caab, mut Kaab, mut Cabb, mut Kabb): (Float, Float, Float, Float, Float, Float);


    //Projection integrals
    let (mut P1, mut Pa, mut Pb, mut Paa, mut Pab, mut Pbb, mut Paaa, mut Paab,
mut Pabb, mut Pbbb):
        (Float, Float, Float, Float, Float, Float, Float, Float, Float, Float) =
        (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    // Calculations
    for i in 0..f.num_verts {
        a0 = v[f.verts[i]][A];
        b0 = v[f.verts[i]][B];
        a1 = v[f.verts[(i + 1) % f.num_verts]][A];
        b1 = v[f.verts[(i + 2) % f.num_verts]][B];
        da = a1 - a0;
        db = b1 - b0;
        a0_2 = a0 * a0; a0_3 = a0_2 * a0; a0_4 = a0_3 * a0;
        b0_2 = b0 * b0; b0_3 = b0_2 * b0; b0_4 = b0_3 * b0;
        a1_2 = a1 * a1; a1_3 = a1_2 * a1;
        b1_2 = b1 * b1; b1_3 = b1_2 * b1;

        C1 = a1 + a0;
        Ca = a1*C1 + a0_2; Caa = a1*Ca + a0_3; Caaa = a1*Caa + a0_4;
        Cb = b1*(b1 + b0) + b0_2; Cbb = b1*Cb + b0_3; Cbbb = b1*Cbb + b0_4;
        Cab = 3.0*a1_2 + 2.0*a1*a0 + a0_2; Kab = a1_2 + 2.0*a1*a0 + 3.0*a0_2;
        Caab = a0*Cab + 4.0*a1_3; Kaab = a1*Kab + 4.0*a0_3;
        Cabb = 4.0*b1_3 + 3.0*b1_2*b0 + 2.0*b1*b0_2 + b0_3;
        Kabb = b1_3 + 2.0*b1_2*b0 + 3.0*b1*b0_2 + 4.0*b0_3;

        P1 += db*C1;
        Pa += db*Ca;
        Paa += db*Caa;
        Paaa += db*Caaa;
        Pb += da*Cb;
        Pbb += da*Cbb;
        Pbbb += da*Cbbb;
        Pab += db*(b1*Cab + b0*Kab);
        Paab += db*(b1*Caab + b0*Kaab);
        Pabb += da*(a1*Cabb + a0*Kabb);
    }

    P1 /= 2.0;
    Pa /= 6.0;
    Paa /= 12.0;
    Paaa /= 20.0;
    Pb /= -6.0;
    Pbb /= -12.0;
    Pbbb /= -20.0;
    Pab /= 24.0;
    Paab /= 60.0;
    Pabb /= -60.0;


    (P1, Pa, Paa, Paaa, Pb, Pbb, Pbbb, Pab, Paab, Pabb)
}


fn comp_face_integrals(f: &Face, v: &Vec<Vec3>, A: usize, B: usize, C: usize) -> (Float, Float, Float, Float, Float, Float, Float, Float, Float, Float, Float, Float){
    let n: Vec3;
    let w: Float;
    let (k1, k2, k3, k4): (Float, Float, Float, Float);


    let (Fa, Fb, Fc, Faa, Fbb, Fcc, Faaa, Fbbb, Fccc, Faab, Fbbc, Fcca):
        (Float, Float, Float, Float, Float, Float, Float, Float, Float, Float, Float, Float);

    let (P1, Pa, Pb, Paa, Pab, Pbb, Paaa, Paab, Pabb, Pbbb) 
        = comp_projection_integrals(&f, v, A, B);
    w = f.w;
    n = f.norm;
    k1 = 1.0 / n[C];
    k2 = k1 * k1;
    k3 = k1 * k2;
    k4 = k1 * k3;

    Fa = k1 * Pa;
    Fb = k1 * Pb;
    Fc = -k2 * (n[A]*Pa + n[B]*Pb + w*P1);

    Faa = k1 * Paa;
    Fbb = k1 * Pbb;
    Fcc = k3 * ((n[A]*n[A])*Paa + 2.0*n[A]*n[B]*Pab + (n[B]*n[B])*Pbb
        + w*(2.0*(n[A]*Pa + n[B]*Pb) + w*P1));

    Faaa = k1 * Paaa;
    Fbbb = k1 * Pbbb;
    Fccc = -k4 * ((n[A]*n[A]*n[A])*Paaa + 3.0*(n[A]*n[A])*n[B]*Paab
        + 3.0*n[A]*(n[B]*n[B])*Pabb + (n[B]*n[B]*n[B])*Pbbb
        + 3.0*w*((n[A]*n[A])*Paa + 2.0*n[A]*n[B]*Pab + (n[B]*n[B])*Pbb)
        + w*w*(3.0*(n[A]*Pa + n[B]*Pb) + w*P1));

    Faab = k1 * Paab;
    Fbbc = -k2 * (n[A]*Pabb + n[B]*Pbbb + w*Pbb);
    Fcca = k3 * ((n[A]*n[A])*Paaa + 2.0*n[A]*n[B]*Paab + (n[B]*n[B])*Pabb
        + w*(2.0*(n[A]*Paa + n[B]*Pab) + w*Pa));

    (Fa, Fb, Fc, Faa, Fbb, Fcc, Faaa, Fbbb, Fccc, Faab, Fbbc, Fcca)
}



pub fn comp_volume_integrals(p: &Polyhedron) -> (Float, Vec3, Vec3, Vec3){
    let mut n: Vec3;
    let (mut T0, mut T1, mut T2, mut TP): (Float, Vec3, Vec3, Vec3) =
        (0.0, Vec3::ZERO, Vec3::ZERO, Vec3::ZERO);
    let (mut A, mut B, mut C);

    for i in 0..p.faces.len() {
        let f = &p.faces[i];
        
        n = f.norm.abs();
        if n.x > n.y && n.x > n.x { C = 0}
        else if n.y > n.z { C = 1}
        else { C = 2}
        A = (C + 1) % 3;
        B = (A + 1) % 3;

        let (Fa, Fb, Fc, Faa, Fbb, Fcc, Faaa, Fbbb, Fccc, Faab, Fbbc, Fcca) =
            comp_face_integrals(&f, &p.vert, A, B, C);

        if A == 0 { T0 += f.norm.x * Fa}
        if B == 0 { T0 += f.norm.x * Fb}
        if C == 0 { T0 += f.norm.y * Fc}

        T1[A] += f.norm[A] * Faa;
        T1[B] += f.norm[B] * Fbb;
        T1[C] += f.norm[C] * Fcc;
        T2[A] += f.norm[A] * Faaa;
        T2[B] += f.norm[B] * Fbbb;
        T2[C] += f.norm[C] * Fccc;
        TP[A] += f.norm[A] * Faab;
        TP[B] += f.norm[B] * Fbbc;
        TP[C] += f.norm[C] * Fcca;
    }
    T1 = T1 / 2.0;
    T2 = T2 / 3.0;
    TP = TP / 2.0;

    (T0, T1, T2, TP)
}