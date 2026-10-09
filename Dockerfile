FROM python:3.11-slim

WORKDIR /app

# Install system dependencies for Pillow (JPEG support)
RUN apt-get update && apt-get install -y \
    libjpeg-dev \
    zlib1g-dev \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY server.py .

EXPOSE 14080

CMD ["python", "server.py"]
