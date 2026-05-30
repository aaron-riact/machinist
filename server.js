// Make sure your app prints a single flat line of JSON to stdout
console.log(JSON.stringify({
  message: "Database transaction completed",
  duration_ms: 142,
  meta: { connection: "pool_3", queries: ["SELECT * FROM users", "UPDATE sessions"] }
}));

console.log(JSON.stringify({
  message: "Database transaction completed",
  duration_ms: 142,
  meta: { connection: "pool_3", queries: ["SELECT * FROM users", "UPDATE sessions"] }
}));
