import itertools
import numpy as np
from embedding_utils import create_input_vectors_for_task_4
from sklearn.model_selection import train_test_split
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import accuracy_score
from collections import defaultdict, Counter

# Tüm kombinasyonlar için modeli eğiten ve en iyi sonucu veren kombinasyonu döndüren fonksiyon
def train_all(s_embeddings, g_embeddings, d_embeddings, y):

    # Kullanılacak vektör anahtarları
    vector_keys = [
        "s", "g", "d",
        "s_g", "s_d", "g_d",
        "abs_s_g", "abs_s_d", "abs_g_d", "abs_s_g_d"
    ]

    # Görev için giriş vektörlerini oluştur
    vectors = create_input_vectors_for_task_4(s_embeddings, g_embeddings, d_embeddings)

    best_acc = 0  # En iyi doğruluğu tutacak değişken
    best_combo = None  # En iyi kombinasyonu tutacak değişken

    # 3 ile 10 arasında kombinasyon uzunluklarıyla kombinasyonları oluştur
    for r in range(3, 11):
        # Vektör anahtarlarıyla tüm benzersiz kombinasyonları oluştur
        unique_combos = set(itertools.combinations(vector_keys, r))

        # Her bir kombinasyon için model eğitimi ve testini yap
        for combo in unique_combos:
            combo_list = sorted(list(combo))  # Kombinasyonu sıralı hale getir

            try:
                # Kombinasyona göre vektörleri birleştir (eksik anahtarlar varsa atla)
                X = np.concatenate([vectors[k] for k in combo_list], axis=1)
            except KeyError:
                continue  # Eğer kombinasyonda eksik bir anahtar varsa, geç

            # Veriyi eğitim ve test olarak ayır
            X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

            # Random Forest sınıflandırıcısını başlat ve eğit
            clf = RandomForestClassifier(n_estimators=50, random_state=42)
            clf.fit(X_train, y_train)

            # Test verisiyle tahmin yap
            y_pred = clf.predict(X_test)

            # Doğruluk hesapla
            acc = accuracy_score(y_test, y_pred)

            # Eğer bu doğruluk, şu ana kadar elde edilen en yüksek doğruluktan yüksekse, güncelle
            if acc > best_acc:
                best_acc = acc
                best_combo = combo_list  # En iyi kombinasyonu kaydet

    # En iyi kombinasyonu döndür
    return best_combo
