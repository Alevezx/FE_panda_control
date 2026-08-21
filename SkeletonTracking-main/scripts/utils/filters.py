#!/usr/bin/env python3

"""
░█▀▀░▀█▀░█░░░▀█▀░█▀▀░█▀▄░█▀▀
░█▀▀░░█░░█░░░░█░░█▀▀░█▀▄░▀▀█
░▀░░░▀▀▀░▀▀▀░░▀░░▀▀▀░▀░▀░▀▀▀

Script for filtering and smoothing 3D keypoints of human skeletons, using One Euro Filter 
for temporal smoothing and occlusion handling.
Includes a robust depth reading function that computes the median depth in a neighborhood 
to handle noise and missing data from the RealSense camera.
"""

import time
import math
import numpy as np



# Class implementing the One Euro Filter for smoothing 1D signals, adapted for 3D keypoints in Keypoints3DSmoother
class OneEuroFilter:
    # Costruttore del filtro
    def __init__(self, t0, x0, dx0=0.0, min_cutoff=1.0, beta=0.0, d_cutoff=1.0):
        self.min_cutoff = float(min_cutoff)  # Cutoff minimo per segnali quasi fermi
        self.beta = float(beta)              # Coefficiente di adattamento alla velocità
        self.d_cutoff = float(d_cutoff)      # Cutoff per la derivata (velocità)
        self.x_prev = float(x0)              # Ultimo valore filtrato
        self.dx_prev = float(dx0)            # Ultima velocità stimata
        self.t_prev = float(t0)              # Timestamp ultimo aggiornamento

    # Calcola il fattore di smoothing basato su tempo e cutoff
    def smoothing_factor(self, t_e, cutoff):
        r = 2.0 * math.pi * cutoff * t_e
        return r / (r + 1.0)

    # Applica smoothing esponenziale
    def exponential_smoothing(self, alpha, x, x_prev):
        return alpha * x + (1.0 - alpha) * x_prev

    # Aggiorna il filtro con nuovo campione (t, x)
    def __call__(self, t, x):
        t_e = t - self.t_prev  # Tempo trascorso (dt)
        if t_e <= 0.0:
            return self.x_prev
        # Stima velocità con smoothing
        a_d = self.smoothing_factor(t_e, self.d_cutoff)
        dx = (x - self.x_prev) / t_e
        dx_hat = self.exponential_smoothing(a_d, dx, self.dx_prev)
        # Adatta cutoff basato sulla velocità
        cutoff = self.min_cutoff + self.beta * abs(dx_hat)
        # Filtra posizione
        a = self.smoothing_factor(t_e, cutoff)
        x_hat = self.exponential_smoothing(a, x, self.x_prev)
        # Aggiorna stato
        self.x_prev = x_hat
        self.dx_prev = dx_hat
        self.t_prev = t
        return x_hat



# Class to smooth 3D keypoints using One Euro Filter, with occlusion handling (holding last valid position for a short time)
class Keypoints3DSmoother:
    # Costruttore: inizializza parametri e strutture dati
    def __init__(self, num_kpts=17, min_cutoff=0.1, beta=1.0):
        self.num_kpts = num_kpts  # Numero di keypoints (default 17 per COCO pose)
        self.min_cutoff = min_cutoff  # Cutoff minimo per filtri
        self.beta = beta              # Beta per adattamento velocità
        self.t0 = time.monotonic()    # Tempo di riferimento iniziale
        self.initialized = False      # Flag per inizializzazione filtri
        self.filters = []             # Lista di tuple (filter_x, filter_y, filter_z) per keypoint
        self.last_valid = np.full((num_kpts, 3), np.nan, dtype=np.float32)  # Ultimi valori validi
        self.last_valid_time = np.zeros(num_kpts, dtype=np.float64)         # Timestamp ultimi valori validi

    # Metodo di aggiornamento: applica filtri ai nuovi keypoints
    def update(self, xyz, conf, conf_thr):
        t = time.monotonic() - self.t0  # Tempo relativo
        # Inizializzazione lazy dei filtri al primo frame valido
        if not self.initialized:
            for i in range(self.num_kpts):
                x0 = float(xyz[i, 0]) if np.isfinite(xyz[i, 0]) else 0.0
                y0 = float(xyz[i, 1]) if np.isfinite(xyz[i, 1]) else 0.0
                z0 = float(xyz[i, 2]) if np.isfinite(xyz[i, 2]) else 0.0
                self.filters.append((
                    OneEuroFilter(t, x0, min_cutoff=self.min_cutoff, beta=self.beta),
                    OneEuroFilter(t, y0, min_cutoff=self.min_cutoff, beta=self.beta),
                    OneEuroFilter(t, z0, min_cutoff=self.min_cutoff, beta=self.beta),
                ))
            self.initialized = True

        out = np.copy(xyz).astype(np.float32)  # Copia per output
        for i in range(self.num_kpts):
            # Controlla validità del keypoint (confidenza e finitezza)
            valid = (conf[i] >= conf_thr) and np.all(np.isfinite(xyz[i]))
            
            if not valid:
                # Gestione occlusioni: usa ultimi valori validi se recenti (max 0.5s)
                # Se il dato manca per più di 0.5s, smettiamo di predire e restituiamo NaN.
                if np.all(np.isfinite(self.last_valid[i])) and (t - self.last_valid_time[i] < 0.5):
                    out[i] = self.last_valid[i]
                else:
                    out[i] = np.array([np.nan, np.nan, np.nan], dtype=np.float32)
                continue
            # Applica il filtro su ogni asse
            fx, fy, fz = self.filters[i] # nota che fx,fy,fz sono istanze (oggetti) di OneEuroFilter!
            out[i, 0] = fx(t, float(xyz[i, 0]))
            out[i, 1] = fy(t, float(xyz[i, 1]))
            out[i, 2] = fz(t, float(xyz[i, 2]))
            self.last_valid[i] = out[i]
            self.last_valid_time[i] = t
        return out
