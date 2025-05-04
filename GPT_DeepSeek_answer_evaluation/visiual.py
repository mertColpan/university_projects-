import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

# GPT ve DeepSeek modellerinin Top-1 ve Top-5 başarı oranlarını görselleştiren fonksiyon
def visualize_top1_top5(gpt_top1, gpt_top5, deepseek_top1, deepseek_top5, n, title):
    # Başarı oranlarını yüzdelik dilime dönüştür
    gpt_top1_acc = (gpt_top1 / n) * 100
    gpt_top5_acc = (gpt_top5 / n) * 100
    deepseek_top1_acc = (deepseek_top1 / n) * 100
    deepseek_top5_acc = (deepseek_top5 / n) * 100

    # Bar grafiği için etiketler ve başarı oranları
    labels = ['Top-1', 'Top-5']
    gpt_scores = [gpt_top1_acc, gpt_top5_acc]
    deepseek_scores = [deepseek_top1_acc, deepseek_top5_acc]

    # X eksenindeki konumları belirle
    x = range(len(labels))
    width = 0.35  # Bar genişliği

    # Görselleştirme başlat
    fig, ax = plt.subplots()
    # GPT ve DeepSeek için başarı oranlarını bar grafiğinde göster
    ax.bar([i - width/2 for i in x], gpt_scores, width, label='GPT', color='blue')
    ax.bar([i + width/2 for i in x], deepseek_scores, width, label='DeepSeek', color='green')

    # Grafik üzerine açıklamalar ekle
    ax.set_ylabel('Başarı Oranı (%)')
    ax.set_title(title)
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.legend()
    plt.ylim(0, 100)  # Y ekseni %0-100 arası olacak
    plt.tight_layout()
    plt.show()

# Sınıf bazında doğru tahminlerin ortalamalarını görselleştiren fonksiyon
def visualize_labels(avg_values):
    # Her bir sınıf için etiketler
    categories = ['Class 1', 'Class 2', 'Class 3', 'Class 4']
    values = avg_values

    # Sınıflar için renk paleti
    colors = ['skyblue', 'lightgreen', 'salmon', 'plum']

    # Bar grafiği oluştur
    plt.figure(figsize=(8, 5))
    bars = plt.bar(categories, values, color=colors)

    # Barların üzerinde değerleri göstermek için döngü
    for bar in bars:
        height = bar.get_height()
        plt.text(bar.get_x() + bar.get_width()/2.0, height + 0.01, f"{height:.2f}", ha='center', va='bottom')

    # Y eksenini logaritmik ölçeğe göre ayarla
    plt.yscale("log")  # Alternatif olarak logaritmik yerine sabit bir değerle de ayarlanabilir
    plt.ylabel('Average per Class (log scale)')
    plt.title('Average Correct Predictions per Class')
    plt.grid(True, axis='y', linestyle='--', alpha=0.5)  # Y ekseni için grid
    plt.tight_layout()
    plt.show()

# Top-1 ve Top-5 başarı oranlarını etiket bazında gösteren fonksiyon
def visualize_final_averages(gpt_top1, gpt_top5, deep_top1, deep_top5):
    # Etiketler
    labels = ['Label 1', 'Label 2', 'Label 3', 'Label 4']
    x = np.arange(len(labels))  # Etiketlerin konumları
    width = 0.18  # Bar genişliği

    # Grafik hazırlığı
    fig, ax = plt.subplots(figsize=(10, 6))

    # Farklı modellerin başarı oranlarını farklı bar'lar ile göster
    bars1 = ax.bar(x - 1.5*width, gpt_top1, width, label='GPT Top-1', color='#1f77b4')
    bars2 = ax.bar(x - 0.5*width, gpt_top5, width, label='GPT Top-5', color='#aec7e8')
    bars3 = ax.bar(x + 0.5*width, deep_top1, width, label='DeepSeek Top-1', color='#ff7f0e')
    bars4 = ax.bar(x + 1.5*width, deep_top5, width, label='DeepSeek Top-5', color='#ffbb78')

    # Başlık ve etiketler
    ax.set_ylabel('Accuracy')
    ax.set_ylim(0, 1.05)  # Y ekseninin aralığı
    ax.set_title('Top-1 and Top-5 Accuracy by Label and Model')
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.legend()
    ax.grid(axis='y', linestyle='--', alpha=0.7)  # Y ekseninde grid

    # Her barın üzerinde değerleri göstermek için anotasyon ekle
    for bars in [bars1, bars2, bars3, bars4]:
        for bar in bars:
            height = bar.get_height()
            ax.annotate(f'{height:.2f}',
                        xy=(bar.get_x() + bar.get_width() / 2, height),
                        xytext=(0, 3),
                        textcoords="offset points",
                        ha='center', va='bottom', fontsize=8)

    plt.tight_layout()
    plt.show()
