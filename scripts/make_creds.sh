#!/usr/bin/env bash
set -e

# -----------------------------
# Config
# -----------------------------
ROOT_CA_NAME="dev-root-ca"
SERVER_CERT_NAME="dev-server"
CERT_DIR="./certs"
DAYS=3650  # Validity in days

# -----------------------------
# Create certs directory
# -----------------------------
mkdir -p "$CERT_DIR"

# -----------------------------
# Detect local IP address
# -----------------------------
LOCAL_IP=$(hostname -I | awk '{print $1}')
echo "Detected local IP: $LOCAL_IP"

# -----------------------------
# Generate Root CA
# -----------------------------
echo "Generating root CA..."
openssl genrsa -out "$CERT_DIR/$ROOT_CA_NAME.key" 2048
openssl req -x509 -new -nodes -key "$CERT_DIR/$ROOT_CA_NAME.key" \
  -sha256 -days $DAYS -out "$CERT_DIR/$ROOT_CA_NAME.pem" \
  -subj "/C=US/ST=Local/L=Local/O=Dev/OU=Dev/CN=DevRootCA"

# -----------------------------
# Generate server key & CSR
# -----------------------------
echo "Generating server key & CSR..."
openssl genrsa -out "$CERT_DIR/$SERVER_CERT_NAME.key" 2048
openssl req -new -key "$CERT_DIR/$SERVER_CERT_NAME.key" \
  -out "$CERT_DIR/$SERVER_CERT_NAME.csr" \
  -subj "/C=US/ST=Local/L=Local/O=Dev/OU=Dev/CN=localhost"

# -----------------------------
# Create SAN config file
# -----------------------------
SAN_CONF="$CERT_DIR/san.conf"
cat > "$SAN_CONF" <<EOL
[req]
distinguished_name = req_distinguished_name
req_extensions = v3_req

[req_distinguished_name]

[v3_req]
subjectAltName = @alt_names

[alt_names]
DNS.1 = localhost
IP.1 = 127.0.0.1
IP.2 = $LOCAL_IP
EOL

# -----------------------------
# Generate server certificate signed by root CA
# -----------------------------
echo "Signing server certificate..."
openssl x509 -req -in "$CERT_DIR/$SERVER_CERT_NAME.csr" \
  -CA "$CERT_DIR/$ROOT_CA_NAME.pem" -CAkey "$CERT_DIR/$ROOT_CA_NAME.key" \
  -CAcreateserial -out "$CERT_DIR/$SERVER_CERT_NAME.crt" \
  -days $DAYS -sha256 -extfile "$SAN_CONF" -extensions v3_req

# -----------------------------
# Done
# -----------------------------
echo "✅ Certificates generated in $CERT_DIR:"
echo " - Root CA: $CERT_DIR/$ROOT_CA_NAME.pem (import into browser)"
echo " - Server cert: $CERT_DIR/$SERVER_CERT_NAME.crt"
echo " - Server key: $CERT_DIR/$SERVER_CERT_NAME.key"

echo ""
echo "Next steps:"
echo "1. Import $CERT_DIR/$ROOT_CA_NAME.pem into your OS/browser as a trusted CA."
echo "2. Use $CERT_DIR/$SERVER_CERT_NAME.crt and $CERT_DIR/$SERVER_CERT_NAME.key in your WSS server."
