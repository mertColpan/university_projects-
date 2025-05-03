import numpy as np
import matplotlib.pyplot as plt
import random
import argparse
import sys

# Rastgele gen havuzu oluşturur
# size: Her bir pattern'in boyutu (3x3), num_genes: populasyon büyüklüğü, num_patterns_per_gene: bir gende kaç adet pattern var (örn: 7)
def create_random_population(size, num_genes, num_patterns_per_gene):
    return [
        [np.random.randint(0, 2, (size, size)) for _ in range(num_patterns_per_gene)]
        for _ in range(num_genes)
    ]

# Verilen .txt dosyalarından 24x24 boyutundaki hedef imgeleri okur
def read_files(file_names):
    images = []
    for file in file_names:
        with open('./images/' + file, 'r') as f:
            lines = f.readlines()
        images.append(np.array([list(map(int, line.strip())) for line in lines]))
    return images

# Verilen 3x3 image bloğu için gen içerisindeki en iyi pattern ile Hamming mesafesini hesaplar
def find_best_pattern_hamming_distance(gene, image_block, gene_size, pattern_size):
    min_distance = pattern_size * pattern_size  # Maksimum mesafe 9’dur (3x3 için)
    
    for i in range(gene_size):  # Her bir pattern için
        cur_pattern = gene[i]
        cur_distance = np.sum(image_block != cur_pattern)  # Eleman farklarını say

        if min_distance > cur_distance:
            min_distance = cur_distance

    return min_distance

# Fitness fonksiyonu: Bir genin hedef imgeye ne kadar benzediğini toplam Hamming mesafesi ile ölçer (düşük mesafe = yüksek benzerlik)
def fitness_function(gene, image, gene_size, pattern_size):
    total_distance = 0
    for i in range(0, 24, 3):  # 3x3 bloklar halinde gezin
        for j in range(0, 24, 3):
            total_distance += find_best_pattern_hamming_distance(gene, image[i:i+3, j:j+3], gene_size, pattern_size)
    return total_distance

# Seçim yöntemi: Roulette Wheel. Daha iyi fitness’a sahip genlerin seçilme olasılığı daha yüksektir.
def roulette_wheel(genes, total_distance, genes_num):
    valid_distances = np.array(total_distance)
    fitness = 1 / valid_distances
    total_fitness = np.sum(fitness)

    if total_fitness == 0:
        selected_index = np.random.randint(0, len(genes))
        genes.append(genes[selected_index])
        total_distance.append(total_distance[selected_index])
        return

    prob = fitness / total_fitness
    cumulative_prob = np.cumsum(prob)
    new_genes = []
    new_total_distance = []
    for _ in range(genes_num):
        random_value = np.random.rand()
        for i in range(len(genes)):
            if random_value <= cumulative_prob[i]:
                new_genes.append(genes[i])
                new_total_distance.append(total_distance[i])
                break
    return new_genes, new_total_distance

# Seçim yöntemi: Turnuva seçimi. Rastgele k adet gen seçilir, en iyisi kazanır.
def tournament_selection(genes, total_distance, k=2, population=6):
    new_genes = []
    new_total_distance = []
    for i in range(population):
        selected = np.random.choice(len(genes), k, replace=False)
        best_index = selected[np.argmin([total_distance[i] for i in selected])]
        new_genes.append(genes[best_index])
        new_total_distance.append(total_distance[best_index])
    return new_genes, new_total_distance

# Elitizm: En iyi genlerden bir kısmını (elite_size kadar) doğrudan bir sonraki nesle aktarır.
def elitism_selection(genes, total_distance, elite_size=2):
    sorted_indices = np.argsort(total_distance)[:elite_size]
    elite_genes = [genes[i] for i in sorted_indices]
    elite_distances = [total_distance[i] for i in sorted_indices]
    return elite_genes, elite_distances

# Tek noktalı çaprazlama
def one_point_crossover(gen_size, gene1, gene2):
    cross_point = np.random.randint(1, gen_size)
    return gene1[:cross_point] + gene2[cross_point:]

# Çok noktalı çaprazlama
def multi_point_crossover(gen_size, gene1, gene2, points=2):
    points = sorted(np.random.choice(range(1, gen_size), points, replace=False))
    new_gene, toggle, start = [], False, 0
    for point in points + [gen_size]:
        new_gene.extend(gene1[start:point] if toggle else gene2[start:point])
        toggle = not toggle
        start = point
    return new_gene

# Uniform (rastgele) çaprazlama
def uniform_crossover(gen_size, gene1, gene2):
    new_gene = []
    for i in range(gen_size):
        new_gene.append(gene1[i] if np.random.rand() > 0.5 else gene2[i])
    return new_gene

# Mutasyon işlemi: Rastgele bir pattern içindeki bir biti çevirir (0->1, 1->0)
def mutate(gene, mutation_rate=0.01):
    for i in range(len(gene)):
        if np.random.rand() < mutation_rate:
            pattern = np.array(gene[i], dtype=int)
            row, col = np.random.randint(0, 3, size=2)
            pattern[row, col] = 1 - pattern[row, col]
            gene[i] = pattern.tolist()
    return gene

# En iyi geni görsel olarak yazdırmak için yardımcı fonksiyon
def print_best_gene(gene, gene_size=7, pattern_size=3):
    for i, pattern in enumerate(gene):
        print(f"Pattern {i+1}:")
        for row in pattern:
            print(" ".join(map(str, row)))
        print()

# Genetik algoritmayı farklı parametrelerle tekrar tekrar çalıştıran deney fonksiyonu
# selection_method: seçilecek yöntem (elitism, roulette, tournament)
# crossover_method: çaprazlama yöntemi (multi_point, one_point, uniform)
# mutation_rate: mutasyon oranı
# population_size: her nesildeki birey sayısı
# generation_num: toplam nesil sayısı
# image_number: kullanılacak hedef imgenin index'i (0-4 arası)
# repetitions: aynı deneyin kaç kez tekrar edileceği (ortalama almak için)
def run_experiment(selection_method, crossover_method, mutation_rate, population_size, generation_num, image_number, repetitions=10):
    file_names = ["sword.txt", "face.txt", "heart.txt", "diamond.txt", "cross.txt"]
    images = read_files(file_names)
    gene_size = 7
    best_scores = np.zeros((repetitions, generation_num))
    
    for rep in range(repetitions):
        genes = create_random_population(3, population_size, gene_size)
        total_distance = [fitness_function(g, images[image_number], gene_size, 3) for g in genes]
        
        for gen in range(generation_num):
            while True:  
                parent1_idx, parent2_idx = random.sample(range(len(genes)), 2)  
                if total_distance[parent1_idx] != total_distance[parent2_idx]:  
                    break

            parent1 = genes[parent1_idx]
            parent2 = genes[parent2_idx]

            new_genes = []
            if crossover_method == 'multi_point':
                child = multi_point_crossover(gene_size, parent1, parent2)
            elif crossover_method == 'uniform':
                child = uniform_crossover(gene_size, parent1, parent2)
            else:
                child = one_point_crossover(gene_size, parent1, parent2)
            
            new_genes.append(mutate(child, mutation_rate))
        
            genes.extend(new_genes)
            total_distance.extend([fitness_function(g, images[image_number], gene_size, 3) for g in new_genes])

            if selection_method == 'elitism':
                genes, total_distance = elitism_selection(genes, total_distance, 3)
            elif selection_method == 'roulette':
                genes, total_distance = roulette_wheel(genes, total_distance)
            elif selection_method == 'tournament':
                genes, total_distance = tournament_selection(genes, total_distance)

            new_genes = []
            for _ in range(population_size - len(genes)):
                p1, p2 = random.sample(genes, 2)
                child = one_point_crossover( p1, p2)  # gen_size parametresini ekledim
                new_genes.append(mutate(child, mutation_rate))

            genes.extend(new_genes)  # extend kullanımı artık doğru
            best_scores[rep, gen] = min(total_distance)
        print(f'Processing...{rep}')
    return np.mean(best_scores, axis=0)

def parse_arguments():
    parser = argparse.ArgumentParser(description="Genetik algoritma parametreleri")
    parser.add_argument('--populations', type=int, nargs='+', default=[5, 10, 15, 20], 
                        help='Farklı popülasyon büyüklükleri')
    parser.add_argument('--mutation_rates', type=float, nargs='+', default=[0.2, 0.1, 0.05, 0.01],
                        help='Farklı mutasyon oranları')
    parser.add_argument('--generation_num', type=int, default=20, help='Toplam nesil sayısı')
    parser.add_argument('--repetitions', type=int, default=50, help='Deneyin tekrar sayısı')
    parser.add_argument('--selection_method', type=str, default='elitism_selection', 
                        choices=['elitism_selection', 'roulette', 'tournament'], help='Seçim yöntemi')
    parser.add_argument('--crossover_method', type=str, default='one_point', 
                        choices=['one_point', 'multi_point', 'uniform'], help='Çaprazlama yöntemi')
    parser.add_argument('--generation_num_exp', type=int, default=10, help='Deneydeki nesil sayısı')
    parser.add_argument('--image_number', type=int, default=0, help='Kullanılacak hedef imgenin index’i (0-4)')
    
    return parser.parse_args()

def main(selection_method='elitism_selection', 
         crossover_method='one_point', 
         generation_num=20, 
         image_number=0,
         populations=[5, 10, 15, 20], 
         mutation_rates=[0.2, 0.1, 0.05, 0.01], 
         repetitions=50):
    
    results = {}

    # Parametreleri kullanarak farklı kombinasyonlarla deney yapalım
    for pop in populations:
        for mut in mutation_rates:
            # Deneyi çalıştır ve sonuçları kaydet
            avg_fitness = run_experiment(
                selection_method=selection_method, 
                crossover_method=crossover_method, 
                mutation_rate=mut, 
                population_size=pop, 
                generation_num=generation_num, 
                image_number=image_number, 
                repetitions=repetitions
            )
            results[(pop, mut)] = avg_fitness

    # Sonuçları görselleştirelim
    plt.figure(figsize=(10, 6))
    for key, values in results.items():
        plt.plot(values, label=f'Pop: {key[0]}, Mut: {key[1]}')

    plt.xlabel("Generations")
    plt.ylabel("Average Fitness")
    plt.title("Effect of Population Size and Mutation Rate")
    plt.legend()
    plt.show()

if __name__ == "__main__":
    args = parse_arguments()
    
    # Parametreleri main fonksiyonuna gönder
    main(
        selection_method=args.selection_method,
        crossover_method=args.crossover_method,
        mutation_rates=args.mutation_rates,
        generation_num=args.generation_num_exp,
        image_number=args.image_number
    )
    
