from sklearn.linear_model import LogisticRegression
from sklearn.ensemble import RandomForestClassifier
from sklearn.model_selection import train_test_split
from sklearn.metrics import classification_report, accuracy_score
from sklearn.preprocessing import LabelEncoder

# Rastgele Orman sınıflandırıcısı ile model eğitimi yapan fonksiyon
def train_model_with_random_forest(X, y, n_estimators):
    # Veriyi eğitim ve test olarak ayır (test oranı %20)
    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

    # RandomForest sınıflandırıcısını başlat (belirtilen n_estimators ile)
    clf = RandomForestClassifier(n_estimators=n_estimators, random_state=42)
    # Modeli eğitim verileriyle eğit
    clf.fit(X_train, y_train)

    # Test verisi ile tahmin yap
    y_pred = clf.predict(X_test)

    # Başarı oranını ve sınıflandırma raporunu yazdır
    print("Accuracy:", accuracy_score(y_test, y_pred))
    print("Classification Report:\n", classification_report(y_test, y_pred))

    # Eğitilen modeli döndür
    return clf

# Lojistik Regresyon ile model eğitimi yapan fonksiyon
def train_model_with_logistic_regression(X, y):
    # Etiketleri sayısal verilere dönüştürmek için LabelEncoder kullan
    encoder = LabelEncoder()
    y_encoded = encoder.fit_transform(y)

    # Veriyi eğitim ve test olarak ayır (test oranı %20)
    X_train, X_test, y_train, y_test = train_test_split(X, y_encoded, test_size=0.2, random_state=42)

    # Lojistik regresyon modelini başlat (class_weight='balanced' ile dengesiz sınıfları dikkate al)
    model = LogisticRegression(class_weight='balanced', max_iter=1000, random_state=42)
    # Modeli eğitim verileriyle eğit
    model.fit(X_train, y_train)

    # Test verisi ile tahmin yap
    y_pred = model.predict(X_test)

    # Başarı oranını ve sınıflandırma raporunu yazdır
    print("Accuracy:", accuracy_score(y_test, y_pred))
    print("Classification Report:\n", classification_report(y_test, y_pred))
