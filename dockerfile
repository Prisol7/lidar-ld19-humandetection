FROM rust:1.95 AS build
WORKDIR /src
COPY Cargo.toml Cargo.lock ./
COPY src ./src
RUN cargo build --release --bin main

FROM debian:bookworm-slim
RUN apt-get update && apt-get install -y --no-install-recommends \
      libxkbcommon0 libwayland-client0 libwayland-cursor0 \
      libx11-6 libxcursor1 libxrandr2 libxi6 ca-certificates \
    && rm -rf /var/lib/apt/lists/*
COPY --from=build /src/target/release/main /usr/local/bin/lidar
CMD ["/usr/local/bin/lidar"]
