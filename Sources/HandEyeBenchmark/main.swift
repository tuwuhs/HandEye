
import Benchmark

let myBenchmark = BenchmarkSuite(name: "myBenchmark") { suite in
  suite.benchmark(
    "Hello world",
    settings: Iterations(1), TimeUnit(.us)
  ) {
    var x: String = ""
    x.reserveCapacity(2000)
    for _ in 0..<1000 {
      x += "hi"
    }
  }
}

Benchmark.main([
  myBenchmark,
])