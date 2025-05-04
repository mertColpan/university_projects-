import numpy as np
import torch
import torch.nn.functional as F

# Verilen bir metinden embedding (vektör temsili) çıkaran fonksiyon
def get_embedding(text, model, device, tokenizer):
    # Metni tokenize ediyoruz ve cihaz (CPU/GPU) üzerine taşıyoruz
    inputs = tokenizer(text, return_tensors="pt", padding=True, truncation=True).to(device)
    with torch.no_grad():  # Gradients hesabı yapmadan modelden geçiyoruz (daha hızlı ve bellek dostu)
        outputs = model(**inputs)
        # [CLS] token'ının embedding'ini alıyoruz (genellikle tüm cümlenin temsili olarak kullanılır)
        cls_embedding = outputs.last_hidden_state[:, 0]
        # Embedding'i normalize edip CPU'ya taşıyoruz
        return F.normalize(cls_embedding, p=2, dim=1).cpu()

# Task tanımı ve sorguyu birleştirerek yeni bir görev tanımı oluşturan fonksiyon
def get_detailed_instruct(task_description: str, query: str) -> str:
    return f'Instruct: {task_description}\nQuery: {query}'

# Üç farklı metin listesinden embedding'ler oluşturan fonksiyon
def create_vectors(gpt, deep, q, model):
    # Liste elemanlarını string'e çeviriyoruz
    gpt = [str(x) for x in gpt]
    deep = [str(x) for x in deep]
    q = [str(x) for x in q]

    # Metin listelerinin her biri için embedding hesaplıyoruz
    s_embeddings = model.encode(q, convert_to_tensor=True, normalize_embeddings=True, show_progress_bar=True, batch_size=8)
    g_embeddings = model.encode(gpt, convert_to_tensor=True, normalize_embeddings=True, show_progress_bar=True, batch_size=8)
    d_embeddings = model.encode(deep, convert_to_tensor=True, normalize_embeddings=True, show_progress_bar=True, batch_size=8)

    # Üç farklı embedding'i döndürüyoruz
    return s_embeddings, g_embeddings, d_embeddings

# Üç vektörden giriş (input) özellikleri oluşturan fonksiyon (task 3 için)
def create_input_vectors(s, g, d):
    # Tensorları numpy array'lerine çeviriyoruz
    s = s.cpu().numpy()
    g = g.cpu().numpy()
    d = d.cpu().numpy()
    
    # Vektörler arası farkları hesaplıyoruz
    s_g = s - g
    s_d = s - d
    g_d = g - d

    # Vektörler arası farkların mutlak değerlerini alıyoruz
    abs_s_g = abs(s - g)
    abs_s_d = abs(s - d)
    abs_g_d = abs(g - d)

    # İki mutlak farkın da birbirine farkının mutlak değerini alıyoruz
    abs_s_g_d = abs(abs_s_g - abs_s_d)

    # Bütün özellikleri bir araya getiriyoruz
    input_features = np.stack([s, g, d, s_g, s_d, g_d, abs_s_g, abs_s_d, abs_g_d, abs_s_g_d], axis=-1)
    input_features = input_features.reshape(input_features.shape[0], -1)  # Her örneği tek boyutlu hale getiriyoruz

    return input_features  # Makine öğrenmesi modeline verilecek giriş vektörleri

# Üç vektörden ayrı ayrı özellikler üreten fonksiyon (task 4 için)
def create_input_vectors_for_task_4(s, g, d):
    # Tensorları numpy array'lerine çeviriyoruz
    s = s.cpu().numpy()
    g = g.cpu().numpy()
    d = d.cpu().numpy()

    # Her özelliği ayrı ayrı dictionary içinde tutuyoruz
    vectors = {
        "s": s,
        "g": g,
        "d": d,
        "s_g": s - g,  # s ve g arasındaki fark
        "s_d": s - d,  # s ve d arasındaki fark
        "g_d": g - d,  # g ve d arasındaki fark
        "abs_s_g": np.abs(s - g),  # s ve g farkının mutlak değeri
        "abs_s_d": np.abs(s - d),  # s ve d farkının mutlak değeri
        "abs_g_d": np.abs(g - d),  # g ve d farkının mutlak değeri
        "abs_s_g_d": np.abs(np.abs(s - g) - np.abs(s - d))  # iki mutlak farkın farkının mutlak değeri
    }

    return vectors  # Task 4 için giriş vektörlerinin her biri ayrı şekilde tutulur
