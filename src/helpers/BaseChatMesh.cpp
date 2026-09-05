#include <helpers/BaseChatMesh.h>
#include <Utils.h>

#ifndef SERVER_RESPONSE_DELAY
  #define SERVER_RESPONSE_DELAY   300
#endif

#ifndef TXT_ACK_DELAY
  #define TXT_ACK_DELAY     200
#endif

/* ---- clock plausibility ---------------------------------------------------
 *
 * Adverts carry the sender's own idea of the time, and some nodes out there
 * have that badly wrong. Trusting them without bounds is how this node ended
 * up with an RTC reading 2037: a single neighbour advertised a 2037 timestamp,
 * this node adopted it, and since the clock only ever moves forward there was
 * no way back -- every message it sent afterwards was stamped a decade out.
 *
 * The floor is the firmware's own build date: this code cannot legitimately be
 * running before it was compiled. The ceiling floats above whichever is later,
 * the build date or the clock we already hold, so a board deployed for years
 * still accepts real time from its neighbours after a power-on reset wipes the
 * RTC back to the ESP32 default.
 */
/* How far past the time we already hold an advert may be and still be taken
   seriously, and how far past the build date this node's own clock may be
   before it is treated as corrupt at boot. */
#define RTC_MAX_AHEAD_SECS    (2UL * 365 * 24 * 60 * 60)
#define RTC_MAX_RUNTIME_SECS  (3UL * 365 * 24 * 60 * 60)

/* A forward step this small is ordinary drift and is taken on one node's word.
   Anything larger has to be agreed by RTC_QUORUM nodes whose timestamps fall
   within RTC_CLUSTER_SECS of each other -- otherwise one node running days
   fast drags this one with it, permanently, because the clock never descends
   on its own. RTC_RESYNC_SLACK is how far the stored contacts have to disagree
   with the clock at boot before the clock is the one presumed wrong. */
#define RTC_SMALL_STEP_SECS   300UL
#define RTC_CLUSTER_SECS      900UL
#define RTC_QUORUM            3
#define RTC_RESYNC_SLACK      3600UL

/* __DATE__ is "Mmm dd yyyy" -- fixed width, with a space for a leading zero. */
static constexpr int kBuildYear =
    (__DATE__[7] - '0') * 1000 + (__DATE__[8] - '0') * 100 +
    (__DATE__[9] - '0') * 10 + (__DATE__[10] - '0');
static constexpr int kBuildMonth =
    (__DATE__[0] == 'J' && __DATE__[1] == 'a') ? 1 :
    (__DATE__[0] == 'F') ? 2 :
    (__DATE__[0] == 'M' && __DATE__[2] == 'r') ? 3 :
    (__DATE__[0] == 'A' && __DATE__[1] == 'p') ? 4 :
    (__DATE__[0] == 'M') ? 5 :
    (__DATE__[0] == 'J' && __DATE__[2] == 'n') ? 6 :
    (__DATE__[0] == 'J') ? 7 :
    (__DATE__[0] == 'A') ? 8 :
    (__DATE__[0] == 'S') ? 9 :
    (__DATE__[0] == 'O') ? 10 :
    (__DATE__[0] == 'N') ? 11 : 12;
static constexpr int kBuildDay =
    (__DATE__[4] == ' ' ? 0 : (__DATE__[4] - '0') * 10) + (__DATE__[5] - '0');

/* days_from_civil -- Howard Hinnant's calendar algorithm. Split into one-line
   helpers because C++11 constexpr allows a single return statement per
   function, and this file is built as C++11. */
static constexpr long civil_year(long y, unsigned m) { return y - (m <= 2 ? 1 : 0); }
static constexpr long civil_era(long y) { return (y >= 0 ? y : y - 399) / 400; }
static constexpr unsigned civil_yoe(long y) { return (unsigned)(y - civil_era(y) * 400); }
static constexpr unsigned civil_doy(unsigned m, unsigned d) {
  return (153u * (m > 2 ? m - 3u : m + 9u) + 2u) / 5u + d - 1u;
}
static constexpr unsigned civil_doe(long y, unsigned m, unsigned d) {
  return civil_yoe(y) * 365u + civil_yoe(y) / 4u - civil_yoe(y) / 100u + civil_doy(m, d);
}
static constexpr long days_from_civil(long y, unsigned m, unsigned d) {
  return civil_era(civil_year(y, m)) * 146097L
       + (long)civil_doe(civil_year(y, m), m, d) - 719468L;
}

static constexpr uint32_t kFirmwareBuildTime =
    (uint32_t)(days_from_civil(kBuildYear, (unsigned)kBuildMonth,
                               (unsigned)kBuildDay) * 86400L);

uint32_t BaseChatMesh::clockFloor() const { return kFirmwareBuildTime; }

bool BaseChatMesh::isPlausibleAdvertTime(uint32_t t) {
  uint32_t now = getRTCClock()->getCurrentTime();
  uint32_t base = (now > kFirmwareBuildTime) ? now : kFirmwareBuildTime;
  return t >= kFirmwareBuildTime && t <= base + RTC_MAX_AHEAD_SECS;
}

bool BaseChatMesh::isPlausibleOwnClock(uint32_t t) {
  // Measured against the BUILD DATE, never against the current clock. A bound
  // that floats with the value it is checking can never reject anything, which
  // is exactly how an RTC sitting at 2037 kept passing for plausible.
  return t >= kFirmwareBuildTime &&
         t <= kFirmwareBuildTime + RTC_MAX_RUNTIME_SECS;
}

void BaseChatMesh::sendFloodScoped(const ContactInfo& recipient, mesh::Packet* pkt, uint32_t delay_millis) {
  sendFlood(pkt, delay_millis);
}
void BaseChatMesh::sendFloodScoped(const mesh::GroupChannel& channel, mesh::Packet* pkt, uint32_t delay_millis) {
  sendFlood(pkt, delay_millis);
}

mesh::Packet* BaseChatMesh::createSelfAdvert(const char* name) {
  uint8_t app_data[MAX_ADVERT_DATA_SIZE];
  uint8_t app_data_len;
  {
    AdvertDataBuilder builder(ADV_TYPE_CHAT, name);
    app_data_len = builder.encodeTo(app_data);
  }

  return createAdvert(self_id, app_data, app_data_len);
}

mesh::Packet* BaseChatMesh::createSelfAdvert(const char* name, double lat, double lon) {
  uint8_t app_data[MAX_ADVERT_DATA_SIZE];
  uint8_t app_data_len;
  {
    AdvertDataBuilder builder(ADV_TYPE_CHAT, name, lat, lon);
    app_data_len = builder.encodeTo(app_data);
  }

  return createAdvert(self_id, app_data, app_data_len);
}

void BaseChatMesh::sendAckTo(const ContactInfo& dest, const uint8_t* ack_hash, uint8_t ack_len) {
  if (dest.out_path_len == OUT_PATH_UNKNOWN) {
    mesh::Packet* ack = createAck(ack_hash, ack_len);
    if (ack) sendFloodScoped(dest, ack, TXT_ACK_DELAY);
  } else {
    uint32_t d = TXT_ACK_DELAY;
    if (getExtraAckTransmitCount() > 0) {
      mesh::Packet* a1 = createMultiAck(ack_hash, ack_len, 1);
      if (a1) sendDirect(a1, dest.out_path, dest.out_path_len, d);
      d += 300;
    }

    mesh::Packet* a2 = createAck(ack_hash, ack_len);
    if (a2) sendDirect(a2, dest.out_path, dest.out_path_len, d);
  }
}

/*
 * The newest advert timestamp that at least RTC_QUORUM contacts agree on, to
 * within RTC_CLUSTER_SECS. Ranking alone was not enough: taking the largest
 * let one node with a bad clock decide the time, and taking the Kth largest
 * still lost to any K nodes running fast together. Requiring a CLUSTER means a
 * stray value has to be corroborated by neighbours close to it, and an
 * isolated one is passed over however large it is.
 *
 * O(n^2) over the contact table, run once at boot. At MAX_CONTACTS that is a
 * few thousand comparisons.
 */
uint32_t BaseChatMesh::estimateTimeFromContacts() {
  uint32_t best = 0;

  for (int i = 0; i < num_contacts; i++) {
    uint32_t v = contacts[i].last_advert_timestamp;
    int votes = 0;
    uint32_t lowest = v;

    if (v == 0 || !isPlausibleAdvertTime(v)) continue;
    if (v <= best) continue;                    // cannot improve on what we have

    for (int j = 0; j < num_contacts; j++) {
      uint32_t w = contacts[j].last_advert_timestamp;
      if (w == 0 || !isPlausibleAdvertTime(w)) continue;
      if (w <= v && v - w <= RTC_CLUSTER_SECS) {
        votes++;
        if (w < lowest) lowest = w;
      }
    }
    // Adopt the bottom of the cluster, not its top: erring late is what got
    // this node stuck in the first place.
    if (votes >= RTC_QUORUM) best = lowest;
  }
  return best;
}

void BaseChatMesh::bootstrapRTCfromContacts() {
  // A clock already outside the plausible window is worse than no clock: it
  // makes every neighbour look "behind" and so can never be corrected. Pull it
  // back to the build date first, then let the contacts raise it from there.
  // The value survives a reflash, so it has to be fixed at boot.
  if (!isPlausibleOwnClock(getRTCClock()->getCurrentTime())) {
    getRTCClock()->setCurrentTime(clockFloor());
  }

  uint32_t est = estimateTimeFromContacts();
  if (est == 0) return;

  uint32_t now = getRTCClock()->getCurrentTime();
  // Boot is the one safe moment to step BACKWARDS. Without it a clock that
  // once ran ahead stays ahead for good, since every later advert then looks
  // like the past. Only a corroborated estimate that disagrees by more than an
  // hour is allowed to do it, so ordinary staleness never drags the clock down.
  if (est > now || now - est > RTC_RESYNC_SLACK) {
    getRTCClock()->setCurrentTime(est + 1);
  }
}

/*
 * Runtime adoption. Small forward corrections are taken on sight; a large jump
 * has to be voted for. A lone node running days fast keeps replacing the
 * pending candidate and never reaches quorum, while several correctly-set
 * neighbours cluster and win within a few adverts on a busy mesh.
 */
void BaseChatMesh::setClockAuthoritative(uint32_t secs) {
  getRTCClock()->setCurrentTime(secs);
  _clock_authoritative = true;
  _clock_vote_count = 0;
}

void BaseChatMesh::considerAdvertTime(uint32_t timestamp) {
  uint32_t now = getRTCClock()->getCurrentTime();

  /*
   * Once a host has told us the real time, the mesh does not get a say. Taking
   * time from adverts is a fallback for a node that has never been set, not an
   * ongoing correction: a good part of any mesh runs fast, those nodes
   * corroborate each other, and adopting their consensus walks the clock
   * steadily forward with no way back.
   */
  if (_clock_authoritative) return;

  if (timestamp <= now || !isPlausibleAdvertTime(timestamp)) return;

  if (timestamp - now <= RTC_SMALL_STEP_SECS) {
    getRTCClock()->setCurrentTime(timestamp + 1);
    return;
  }

  uint32_t spread = (timestamp > _clock_vote_time)
                        ? timestamp - _clock_vote_time
                        : _clock_vote_time - timestamp;
  if (_clock_vote_count != 0 && spread <= RTC_CLUSTER_SECS) {
    _clock_vote_count++;
    if (timestamp < _clock_vote_time) _clock_vote_time = timestamp;  // the low end
  } else {
    _clock_vote_time = timestamp;
    _clock_vote_count = 1;
  }
  if (_clock_vote_count >= RTC_QUORUM) {
    getRTCClock()->setCurrentTime(_clock_vote_time + 1);
    _clock_vote_count = 0;
  }
}

ContactInfo* BaseChatMesh::allocateContactSlot(bool transient_only) {
  if (num_contacts < MAX_CONTACTS) {
    return &contacts[num_contacts++];
  } else if (transient_only || shouldOverwriteWhenFull()) {
    // Find oldest non-favourite contact by oldest lastmod timestamp
    int oldest_idx = -1;
    uint32_t oldest_lastmod = 0xFFFFFFFF;
    for (int i = 0; i < num_contacts; i++) {
      if (transient_only) {
        if (contacts[i].type == ADV_TYPE_NONE && contacts[i].lastmod < oldest_lastmod) {
          oldest_lastmod = contacts[i].lastmod;
          oldest_idx = i;
        }
      } else {
        bool is_favourite = (contacts[i].flags & 0x01) != 0;
        if (!is_favourite && contacts[i].lastmod < oldest_lastmod && contacts[i].type != ADV_TYPE_NONE) {
          oldest_lastmod = contacts[i].lastmod;
          oldest_idx = i;
        }
      }
    }
    if (oldest_idx >= 0) {
      onContactOverwrite(contacts[oldest_idx].id.pub_key);
      return &contacts[oldest_idx];
    }
  }
  return NULL; // no space, no overwrite or all contacts are all favourites
}

void BaseChatMesh::populateContactFromAdvert(ContactInfo& ci, const mesh::Identity& id, const AdvertDataParser& parser, uint32_t timestamp) {
  memset(&ci, 0, sizeof(ci));
  ci.id = id;
  ci.out_path_len = OUT_PATH_UNKNOWN;
  StrHelper::strncpy(ci.name, parser.getName(), sizeof(ci.name));
  ci.type = parser.getType();
  if (parser.hasLatLon()) {
    ci.gps_lat = parser.getIntLat();
    ci.gps_lon = parser.getIntLon();
  }
  ci.last_advert_timestamp = timestamp;
  ci.lastmod = getRTCClock()->getCurrentTime();
}

void BaseChatMesh::onAdvertRecv(mesh::Packet* packet, const mesh::Identity& id, uint32_t timestamp, const uint8_t* app_data, size_t app_data_len) {
  AdvertDataParser parser(app_data, app_data_len);
  if (!(parser.isValid() && parser.hasName())) {
    MESH_DEBUG_PRINTLN("onAdvertRecv: invalid app_data, or name is missing: len=%d", app_data_len);
    return;
  }

  ContactInfo* from = NULL;
  for (int i = 0; i < num_contacts; i++) {
    if (id.matches(contacts[i].id)) {  // is from one of our contacts
      from = &contacts[i];
      if (timestamp <= from->last_advert_timestamp) {  // check for replay attacks!!
        MESH_DEBUG_PRINTLN("onAdvertRecv: Possible replay attack, name: %s", from->name);
        return;
      }
      break;
    }
  }

  // save a copy of raw advert packet (to support "Share..." function)
  int plen;
  {
    uint8_t save = packet->header;
    packet->header &= ~PH_ROUTE_MASK;
    packet->header |= ROUTE_TYPE_FLOOD;   // make sure transport codes are NOT saved
    plen = packet->writeTo(temp_buf);
    packet->header = save;
  }

  bool is_new = false; // true = not in contacts[], false = exists in contacts[]
  if (from == NULL) {
    if (!shouldAutoAddContactType(parser.getType())) {
      ContactInfo ci;
      populateContactFromAdvert(ci, id, parser, timestamp);
      onDiscoveredContact(ci, true, packet->path_len, packet->path);       // let UI know
      return;
    }

    // check hop limit for new contacts (0 = no limit, 1 = direct (0 hops), N = up to N-1 hops)
    uint8_t max_hops = getAutoAddMaxHops();
    if (max_hops > 0 && packet->getPathHashCount() >= max_hops) {
      ContactInfo ci;
      populateContactFromAdvert(ci, id, parser, timestamp);
      onDiscoveredContact(ci, true, packet->path_len, packet->path);       // let UI know
      return;
    }

    from = allocateContactSlot();
    if (from == NULL) {
      ContactInfo ci;
      populateContactFromAdvert(ci, id, parser, timestamp);
      onDiscoveredContact(ci, true, packet->path_len, packet->path);
      onContactsFull();
      MESH_DEBUG_PRINTLN("onAdvertRecv: unable to allocate contact slot for new contact");
      return;
    }
    
    populateContactFromAdvert(*from, id, parser, timestamp);
    from->sync_since = 0;
    from->shared_secret_valid = false;
  }

  // update
  putBlobByKey(id.pub_key, PUB_KEY_SIZE, temp_buf, plen);
  StrHelper::strncpy(from->name, parser.getName(), sizeof(from->name));
  from->type = parser.getType();
  if (parser.hasLatLon()) {
    from->gps_lat = parser.getIntLat();
    from->gps_lon = parser.getIntLon();
  }
  from->last_advert_timestamp = timestamp;
  considerAdvertTime(timestamp);
  from->lastmod = getRTCClock()->getCurrentTime();

  onDiscoveredContact(*from, is_new, packet->path_len, packet->path);       // let UI know
}

int BaseChatMesh::searchPeersByHash(const uint8_t* hash) {
  int n = 0;
  for (int i = 0; i < num_contacts && n < MAX_SEARCH_RESULTS; i++) {
    if (contacts[i].id.isHashMatch(hash)) {
      matching_peer_indexes[n++] = i;  // store the INDEXES of matching contacts (for subsequent 'peer' methods)
    }
  }
  return n;
}

void BaseChatMesh::getPeerSharedSecret(uint8_t* dest_secret, int peer_idx) {
  int i = matching_peer_indexes[peer_idx];
  if (i >= 0 && i < num_contacts) {
    memcpy(dest_secret, contacts[i].getSharedSecret(self_id), PUB_KEY_SIZE);
  } else {
    MESH_DEBUG_PRINTLN("getPeerSharedSecret: Invalid peer idx: %d", i);
  }
}

void BaseChatMesh::onPeerDataRecv(mesh::Packet* packet, uint8_t type, int sender_idx, const uint8_t* secret, uint8_t* data, size_t len) {
  int i = matching_peer_indexes[sender_idx];
  if (i < 0 || i >= num_contacts) {
    MESH_DEBUG_PRINTLN("onPeerDataRecv: Invalid sender idx: %d", i);
    return;
  }

  ContactInfo& from = contacts[i];

  if (type == PAYLOAD_TYPE_TXT_MSG && len > 5) {
    uint32_t timestamp;
    memcpy(&timestamp, data, 4);  // timestamp (by sender's RTC clock - which could be wrong)
    uint8_t flags = data[4] >> 2;   // message attempt number, and other flags

    // len can be > original length, but 'text' will be padded with zeroes
    data[len] = 0; // need to make a C string again, with null terminator

    if (flags == TXT_TYPE_PLAIN) {
      from.lastmod = getRTCClock()->getCurrentTime(); // update last heard time
      onMessageRecv(from, packet, timestamp, (const char *) &data[5]);  // let UI know

      int text_len = strlen((char *)&data[5]);
      uint8_t ack_hash[6];    // calc truncated hash of the message timestamp + text + sender pub_key, to prove to sender that we got it
      mesh::Utils::sha256(ack_hash, 4, data, 5 + text_len, from.id.pub_key, PUB_KEY_SIZE);
      // NEW: append (potential) extended attempt byte (to make packethash unique)
      ack_hash[4] = data[5 + text_len + 1];
      getRNG()->random(&ack_hash[5], 1);  // make 6th byte random

      if (packet->isRouteFlood()) {
        // let this sender know path TO here, so they can use sendDirect(), and ALSO encode the ACK
        mesh::Packet* path = createPathReturn(from.id, secret, packet->path, packet->path_len,
                                                PAYLOAD_TYPE_ACK, (uint8_t *) &ack_hash, 6);
        if (path) sendFloodScoped(from, path, TXT_ACK_DELAY);
      } else {
        sendAckTo(from, ack_hash, 6);
      }
    } else if (flags == TXT_TYPE_CLI_DATA) {
      onCommandDataRecv(from, packet, timestamp, (const char *) &data[5]);  // let UI know
      // NOTE: no ack expected for CLI_DATA replies

      if (packet->isRouteFlood()) {
        // let this sender know path TO here, so they can use sendDirect() (NOTE: no ACK as extra)
        mesh::Packet* path = createPathReturn(from.id, secret, packet->path, packet->path_len, 0, NULL, 0);
        if (path) sendFloodScoped(from, path);
      }
    } else if (flags == TXT_TYPE_SIGNED_PLAIN) {
      if (timestamp > from.sync_since) {  // make sure 'sync_since' is up-to-date
        from.sync_since = timestamp;
      }
      from.lastmod = getRTCClock()->getCurrentTime(); // update last heard time
      onSignedMessageRecv(from, packet, timestamp, &data[5], (const char *) &data[9]);  // let UI know

      uint32_t ack_hash;    // calc truncated hash of the message timestamp + text + OUR pub_key, to prove to sender that we got it
      mesh::Utils::sha256((uint8_t *) &ack_hash, 4, data, 9 + strlen((char *)&data[9]), self_id.pub_key, PUB_KEY_SIZE);

      if (packet->isRouteFlood()) {
        // let this sender know path TO here, so they can use sendDirect(), and ALSO encode the ACK
        mesh::Packet* path = createPathReturn(from.id, secret, packet->path, packet->path_len,
                                                PAYLOAD_TYPE_ACK, (uint8_t *) &ack_hash, 4);
        if (path) sendFloodScoped(from, path, TXT_ACK_DELAY);
      } else {
        sendAckTo(from, (uint8_t *) &ack_hash);
      }
    } else {
      MESH_DEBUG_PRINTLN("onPeerDataRecv: unsupported message type: %u", (uint32_t) flags);
    }
  } else if (type == PAYLOAD_TYPE_REQ && len > 4) {
    uint32_t sender_timestamp;
    memcpy(&sender_timestamp, data, 4);
    uint8_t reply_len = onContactRequest(from, sender_timestamp, &data[4], len - 4, temp_buf);
    if (reply_len > 0) {
      if (packet->isRouteFlood()) {
        // let this sender know path TO here, so they can use sendDirect(), and ALSO encode the response
        mesh::Packet* path = createPathReturn(from.id, secret, packet->path, packet->path_len,
                                              PAYLOAD_TYPE_RESPONSE, temp_buf, reply_len);
        if (path) sendFloodScoped(from, path, SERVER_RESPONSE_DELAY);
      } else {
        mesh::Packet* reply = createDatagram(PAYLOAD_TYPE_RESPONSE, from.id, secret, temp_buf, reply_len);
        if (reply) {
          if (from.out_path_len != OUT_PATH_UNKNOWN) {  // we have an out_path, so send DIRECT
            sendDirect(reply, from.out_path, from.out_path_len, SERVER_RESPONSE_DELAY);
          } else {
            sendFloodScoped(from, reply, SERVER_RESPONSE_DELAY);
          }
        }
      }
    }
  } else if (type == PAYLOAD_TYPE_RESPONSE && len > 0) {
    onContactResponse(from, data, len);
    if (packet->isRouteFlood() && from.out_path_len != OUT_PATH_UNKNOWN) {
      // we have direct path, but other node is still sending flood response, so maybe they didn't receive reciprocal path properly(?)
      handleReturnPathRetry(from, packet->path, packet->path_len);
    }
  }
}

bool BaseChatMesh::onPeerPathRecv(mesh::Packet* packet, int sender_idx, const uint8_t* secret, uint8_t* path, uint8_t path_len, uint8_t extra_type, uint8_t* extra, uint8_t extra_len) {
  int i = matching_peer_indexes[sender_idx];
  if (i < 0 || i >= num_contacts) {
    MESH_DEBUG_PRINTLN("onPeerPathRecv: Invalid sender idx: %d", i);
    return false;
  }

  ContactInfo& from = contacts[i];

  return onContactPathRecv(from, packet->path, packet->path_len, path, path_len, extra_type, extra, extra_len);
}

bool BaseChatMesh::onContactPathRecv(ContactInfo& from, uint8_t* in_path, uint8_t in_path_len, uint8_t* out_path, uint8_t out_path_len, uint8_t extra_type, uint8_t* extra, uint8_t extra_len) {
  // NOTE: default impl, we just replace the current 'out_path' regardless, whenever sender sends us a new out_path.
  // FUTURE: could store multiple out_paths per contact, and try to find which is the 'best'(?)
  from.out_path_len = mesh::Packet::copyPath(from.out_path, out_path, out_path_len);  // store a copy of path, for sendDirect()
  from.lastmod = getRTCClock()->getCurrentTime();

  onContactPathUpdated(from);

  if (extra_type == PAYLOAD_TYPE_ACK && extra_len >= 4) {
    // also got an encoded ACK!
    if (processAck(extra) != NULL) {
      txt_send_timeout = 0;   // matched one we're waiting for, cancel timeout timer
    }
  } else if (extra_type == PAYLOAD_TYPE_RESPONSE && extra_len > 0) {
    onContactResponse(from, extra, extra_len);
  }
  return true;  // send reciprocal path if necessary
}

void BaseChatMesh::onAckRecv(mesh::Packet* packet, uint32_t ack_crc) {
  ContactInfo* from;
  if ((from = processAck((uint8_t *)&ack_crc)) != NULL) {
    txt_send_timeout = 0;   // matched one we're waiting for, cancel timeout timer
    packet->markDoNotRetransmit();   // ACK was for this node, so don't retransmit

    if (packet->isRouteFlood() && from->out_path_len != OUT_PATH_UNKNOWN) {
      // we have direct path, but other node is still sending flood, so maybe they didn't receive reciprocal path properly(?)
      handleReturnPathRetry(*from, packet->path, packet->path_len);
    }
  }
}

void BaseChatMesh::handleReturnPathRetry(const ContactInfo& contact, const uint8_t* path, uint8_t path_len) {
  // NOTE: simplest impl is just to re-send a reciprocal return path to sender (DIRECTLY)
  //        override this method in various firmwares, if there's a better strategy
  mesh::Packet* rpath = createPathReturn(contact.id, contact.getSharedSecret(self_id), path, path_len, 0, NULL, 0);
  if (rpath) sendDirect(rpath, contact.out_path, contact.out_path_len, 3000);   // 3 second delay
}

#ifdef MAX_GROUP_CHANNELS
int BaseChatMesh::searchChannelsByHash(const uint8_t* hash, mesh::GroupChannel dest[], int max_matches) {
  int n = 0;
  for (int i = 0; i < MAX_GROUP_CHANNELS && n < max_matches; i++) {
    if (channels[i].channel.hash[0] == hash[0]) {
      dest[n++] = channels[i].channel;
    }
  }
  return n;
}
#endif

void BaseChatMesh::onGroupDataRecv(mesh::Packet* packet, uint8_t type, const mesh::GroupChannel& channel, uint8_t* data, size_t len) {
  if (type == PAYLOAD_TYPE_GRP_TXT) {
    if (len < 5) {
      MESH_DEBUG_PRINTLN("onGroupDataRecv: dropping short group text payload len=%d", (uint32_t)len);
      return;
    }

    uint8_t txt_type = data[4];
    if ((txt_type >> 2) != 0) {
      MESH_DEBUG_PRINTLN("onGroupDataRecv: dropping unsupported group text type=%d", (uint32_t)txt_type);
      return;
    }

    uint32_t timestamp;
    memcpy(&timestamp, data, 4);

    // len can be > original length, but 'text' will be padded with zeroes
    data[len] = 0; // need to make a C string again, with null terminator

    // notify UI  of this new message
    onChannelMessageRecv(channel, packet, timestamp, (const char *) &data[5]);  // let UI know
  } else if (type == PAYLOAD_TYPE_GRP_DATA) {
    if (len < 3) {
      MESH_DEBUG_PRINTLN("onGroupDataRecv: dropping short group data payload len=%d", (uint32_t)len);
      return;
    }

    uint16_t data_type = ((uint16_t)data[0]) | (((uint16_t)data[1]) << 8);
    uint8_t data_len = data[2];
    size_t available_len = len - 3;

    if (data_len > available_len) {
      MESH_DEBUG_PRINTLN("onGroupDataRecv: dropping malformed group data type=%d len=%d available=%d",
                         (uint32_t)data_type, (uint32_t)data_len, (uint32_t)available_len);
      return;
    }

    onChannelDataRecv(channel, packet, data_type, &data[3], data_len);
  }
}

mesh::Packet* BaseChatMesh::composeMsgPacket(const ContactInfo& recipient, uint32_t timestamp, uint8_t attempt, const char *text, uint32_t& expected_ack) {
  int text_len = strlen(text);
  if (text_len > MAX_TEXT_LEN) return NULL;
  if (attempt > 3 && text_len > MAX_TEXT_LEN-2) return NULL;

  uint8_t temp[5+MAX_TEXT_LEN+1];
  memcpy(temp, &timestamp, 4);   // mostly an extra blob to help make packet_hash unique
  temp[4] = (attempt & 3);
  memcpy(&temp[5], text, text_len + 1);

  // calc expected ACK reply
  mesh::Utils::sha256((uint8_t *)&expected_ack, 4, temp, 5 + text_len, self_id.pub_key, PUB_KEY_SIZE);

  int len = 5 + text_len;
  if (attempt > 3) {
    temp[len++] = 0;  // null terminator
    temp[len++] = attempt;  // hide attempt number at tail end of payload
  }

  return createDatagram(PAYLOAD_TYPE_TXT_MSG, recipient.id, recipient.getSharedSecret(self_id), temp, len);
}

int  BaseChatMesh::sendMessage(const ContactInfo& recipient, uint32_t timestamp, uint8_t attempt, const char* text, uint32_t& expected_ack, uint32_t& est_timeout) {
  mesh::Packet* pkt = composeMsgPacket(recipient, timestamp, attempt, text, expected_ack);
  if (pkt == NULL) return MSG_SEND_FAILED;

  uint32_t t = _radio->getEstAirtimeFor(pkt->getRawLength());

  int rc;
  if (recipient.out_path_len == OUT_PATH_UNKNOWN) {
    sendFloodScoped(recipient, pkt);
    txt_send_timeout = futureMillis(est_timeout = calcFloodTimeoutMillisFor(t));
    rc = MSG_SEND_SENT_FLOOD;
  } else {
    sendDirect(pkt, recipient.out_path, recipient.out_path_len);
    txt_send_timeout = futureMillis(est_timeout = calcDirectTimeoutMillisFor(t, recipient.out_path_len));
    rc = MSG_SEND_SENT_DIRECT;
  }
  return rc;
}

int  BaseChatMesh::sendCommandData(const ContactInfo& recipient, uint32_t timestamp, uint8_t attempt, const char* text, uint32_t& est_timeout) {
  int text_len = strlen(text);
  if (text_len > MAX_TEXT_LEN) return MSG_SEND_FAILED;

  uint8_t temp[5+MAX_TEXT_LEN+1];
  memcpy(temp, &timestamp, 4);   // mostly an extra blob to help make packet_hash unique
  temp[4] = (attempt & 3) | (TXT_TYPE_CLI_DATA << 2);
  memcpy(&temp[5], text, text_len + 1);

  auto pkt = createDatagram(PAYLOAD_TYPE_TXT_MSG, recipient.id, recipient.getSharedSecret(self_id), temp, 5 + text_len);
  if (pkt == NULL) return MSG_SEND_FAILED;

  uint32_t t = _radio->getEstAirtimeFor(pkt->getRawLength());
  int rc;
  if (recipient.out_path_len == OUT_PATH_UNKNOWN) {
    sendFloodScoped(recipient, pkt);
    txt_send_timeout = futureMillis(est_timeout = calcFloodTimeoutMillisFor(t));
    rc = MSG_SEND_SENT_FLOOD;
  } else {
    sendDirect(pkt, recipient.out_path, recipient.out_path_len);
    txt_send_timeout = futureMillis(est_timeout = calcDirectTimeoutMillisFor(t, recipient.out_path_len));
    rc = MSG_SEND_SENT_DIRECT;
  }
  return rc;
}

bool BaseChatMesh::sendGroupMessage(uint32_t timestamp, mesh::GroupChannel& channel, const char* sender_name, const char* text, int text_len) {
  uint8_t temp[5+MAX_TEXT_LEN+32];
  memcpy(temp, &timestamp, 4);   // mostly an extra blob to help make packet_hash unique
  temp[4] = 0;  // TXT_TYPE_PLAIN

  sprintf((char *) &temp[5], "%s: ", sender_name);  // <sender>: <msg>
  char *ep = strchr((char *) &temp[5], 0);
  int prefix_len = ep - (char *) &temp[5];

  if (text_len + prefix_len > MAX_TEXT_LEN) text_len = MAX_TEXT_LEN - prefix_len;
  memcpy(ep, text, text_len);
  ep[text_len] = 0;  // null terminator

  auto pkt = createGroupDatagram(PAYLOAD_TYPE_GRP_TXT, channel, temp, 5 + prefix_len + text_len);
  if (pkt) {
    sendFloodScoped(channel, pkt);
    return true;
  }
  return false;
}

bool BaseChatMesh::sendGroupData(mesh::GroupChannel& channel, uint8_t* path, uint8_t path_len, uint16_t data_type, const uint8_t* data, int data_len) {
  if (data_len < 0) {
    MESH_DEBUG_PRINTLN("sendGroupData: invalid negative data_len=%d", data_len);
    return false;
  }
  if (data_len > MAX_GROUP_DATA_LENGTH) {
    MESH_DEBUG_PRINTLN("sendGroupData: data_len=%d exceeds max=%d", data_len, MAX_GROUP_DATA_LENGTH);
    return false;
  }

  uint8_t temp[3 + MAX_GROUP_DATA_LENGTH];
  temp[0] = (uint8_t)(data_type & 0xFF);
  temp[1] = (uint8_t)(data_type >> 8);
  temp[2] = (uint8_t)data_len;
  if (data_len > 0) memcpy(&temp[3], data, data_len);

  auto pkt = createGroupDatagram(PAYLOAD_TYPE_GRP_DATA, channel, temp, 3 + data_len);
  if (pkt == NULL) {
    MESH_DEBUG_PRINTLN("sendGroupData: unable to create group datagram, data_len=%d", data_len);
    return false;
  }

  if (path_len == OUT_PATH_UNKNOWN) {
    sendFloodScoped(channel, pkt);
  } else {
    sendDirect(pkt, path, path_len);
  }

  return true;
}

bool BaseChatMesh::shareContactZeroHop(const ContactInfo& contact) {
  int plen = getBlobByKey(contact.id.pub_key, PUB_KEY_SIZE, temp_buf);  // retrieve last raw advert packet
  if (plen == 0) return false;  // not found

  auto packet = obtainNewPacket();
  if (packet == NULL) return false;  // no Packets available

  packet->readFrom(temp_buf, plen);  // restore Packet from 'blob'
  uint16_t codes[2];
  codes[0] = codes[1] = 0;   // { 0, 0 } means 'send this nowhere'
  sendZeroHop(packet, codes);
  return true;  // success
}

uint8_t BaseChatMesh::exportContact(const ContactInfo& contact, uint8_t dest_buf[]) {
  return getBlobByKey(contact.id.pub_key, PUB_KEY_SIZE, dest_buf);  // retrieve last raw advert packet
}

bool BaseChatMesh::importContact(const uint8_t src_buf[], uint8_t len) {
  auto pkt = obtainNewPacket();
  if (pkt) {
    if (pkt->readFrom(src_buf, len) && pkt->getPayloadType() == PAYLOAD_TYPE_ADVERT) {
      pkt->header |= ROUTE_TYPE_FLOOD;   // simulate it being received flood-mode
      getTables()->clear(pkt);  // remove packet hash from table, so we can receive/process it again
      _pendingLoopback = pkt;  // loop-back, as if received over radio
      return true;  // success
    } else {
      releasePacket(pkt);   // undo the obtainNewPacket()
    }
  }
  return false; // error
}

int BaseChatMesh::sendLogin(const ContactInfo& recipient, const char* password, uint32_t& est_timeout) {
  mesh::Packet* pkt;
  {
    int tlen;
    uint8_t temp[24];
    uint32_t now = getRTCClock()->getCurrentTimeUnique();
    memcpy(temp, &now, 4);   // mostly an extra blob to help make packet_hash unique
    if (recipient.type == ADV_TYPE_ROOM) {
      memcpy(&temp[4], &recipient.sync_since, 4);
      int len = strlen(password); if (len > 15) len = 15;  // max 15 chars currently
      memcpy(&temp[8], password, len);
      tlen = 8 + len;
    } else {
      int len = strlen(password); if (len > 15) len = 15;  // max 15 chars currently
      memcpy(&temp[4], password, len);
      tlen = 4 + len;
    }

    pkt = createAnonDatagram(PAYLOAD_TYPE_ANON_REQ, self_id, recipient.id, recipient.getSharedSecret(self_id), temp, tlen);
  }
  if (pkt) {
    uint32_t t = _radio->getEstAirtimeFor(pkt->getRawLength());
    if (recipient.out_path_len == OUT_PATH_UNKNOWN) {
      sendFloodScoped(recipient, pkt);
      est_timeout = calcFloodTimeoutMillisFor(t);
      return MSG_SEND_SENT_FLOOD;
    } else {
      sendDirect(pkt, recipient.out_path, recipient.out_path_len);
      est_timeout = calcDirectTimeoutMillisFor(t, recipient.out_path_len);
      return MSG_SEND_SENT_DIRECT;
    }
  }
  return MSG_SEND_FAILED;
}

int BaseChatMesh::sendAnonReq(const ContactInfo& recipient, const uint8_t* data, uint8_t len, uint32_t& tag, uint32_t& est_timeout) {
  mesh::Packet* pkt;
  {
    uint8_t temp[MAX_PACKET_PAYLOAD];
    tag = getRTCClock()->getCurrentTimeUnique();
    memcpy(temp, &tag, 4);   // tag to match later (also extra blob to help make packet_hash unique)
    memcpy(&temp[4], data, len);

    pkt = createAnonDatagram(PAYLOAD_TYPE_ANON_REQ, self_id, recipient.id, recipient.getSharedSecret(self_id), temp, 4 + len);
  }
  if (pkt) {
    uint32_t t = _radio->getEstAirtimeFor(pkt->getRawLength());
    if (recipient.out_path_len == OUT_PATH_UNKNOWN) {
      sendFloodScoped(recipient, pkt);
      est_timeout = calcFloodTimeoutMillisFor(t);
      return MSG_SEND_SENT_FLOOD;
    } else {
      sendDirect(pkt, recipient.out_path, recipient.out_path_len);
      est_timeout = calcDirectTimeoutMillisFor(t, recipient.out_path_len);
      return MSG_SEND_SENT_DIRECT;
    }
  }
  return MSG_SEND_FAILED;
}

int  BaseChatMesh::sendRequest(const ContactInfo& recipient, const uint8_t* req_data, uint8_t data_len, uint32_t& tag, uint32_t& est_timeout) {
  if (data_len > MAX_PACKET_PAYLOAD - 16) return MSG_SEND_FAILED;

  mesh::Packet* pkt;
  {
    uint8_t temp[MAX_PACKET_PAYLOAD];
    tag = getRTCClock()->getCurrentTimeUnique();
    memcpy(temp, &tag, 4);   // mostly an extra blob to help make packet_hash unique
    memcpy(&temp[4], req_data, data_len);

    pkt = createDatagram(PAYLOAD_TYPE_REQ, recipient.id, recipient.getSharedSecret(self_id), temp, 4 + data_len);
  }
  if (pkt) {
    uint32_t t = _radio->getEstAirtimeFor(pkt->getRawLength());
    if (recipient.out_path_len == OUT_PATH_UNKNOWN) {
      sendFloodScoped(recipient, pkt);
      est_timeout = calcFloodTimeoutMillisFor(t);
      return MSG_SEND_SENT_FLOOD;
    } else {
      sendDirect(pkt, recipient.out_path, recipient.out_path_len);
      est_timeout = calcDirectTimeoutMillisFor(t, recipient.out_path_len);
      return MSG_SEND_SENT_DIRECT;
    }
  }
  return MSG_SEND_FAILED;
}

int  BaseChatMesh::sendRequest(const ContactInfo& recipient, uint8_t req_type, uint32_t& tag, uint32_t& est_timeout) {
  mesh::Packet* pkt;
  {
    uint8_t temp[13];
    tag = getRTCClock()->getCurrentTimeUnique();
    memcpy(temp, &tag, 4);   // mostly an extra blob to help make packet_hash unique
    temp[4] = req_type;
    memset(&temp[5], 0, 4);  // reserved (possibly for 'since' param)
    getRNG()->random(&temp[9], 4);   // random blob to help make packet-hash unique

    pkt = createDatagram(PAYLOAD_TYPE_REQ, recipient.id, recipient.getSharedSecret(self_id), temp, sizeof(temp));
  }
  if (pkt) {
    uint32_t t = _radio->getEstAirtimeFor(pkt->getRawLength());
    if (recipient.out_path_len == OUT_PATH_UNKNOWN) {
      sendFloodScoped(recipient, pkt);
      est_timeout = calcFloodTimeoutMillisFor(t);
      return MSG_SEND_SENT_FLOOD;
    } else {
      sendDirect(pkt, recipient.out_path, recipient.out_path_len);
      est_timeout = calcDirectTimeoutMillisFor(t, recipient.out_path_len);
      return MSG_SEND_SENT_DIRECT;
    }
  }
  return MSG_SEND_FAILED;
}

bool BaseChatMesh::startConnection(const ContactInfo& contact, uint16_t keep_alive_secs) {
  int use_idx = -1;
  for (int i = 0; i < MAX_CONNECTIONS; i++) {
    if (connections[i].keep_alive_millis == 0) {  // free slot?
      use_idx = i;
    } else if (connections[i].server_id.matches(contact.id)) {  // already in table?
      use_idx = i;
      break;
    }
  }
  if (use_idx < 0) {
    return false;   // table is full
  }
  connections[use_idx].server_id = contact.id;
  uint32_t interval = connections[use_idx].keep_alive_millis = ((uint32_t)keep_alive_secs)*1000;
  connections[use_idx].next_ping = futureMillis(interval);
  connections[use_idx].expected_ack = 0;
  connections[use_idx].last_activity = getRTCClock()->getCurrentTime();
  return true;  // success
}

void BaseChatMesh::stopConnection(const uint8_t* pub_key) {
  for (int i = 0; i < MAX_CONNECTIONS; i++) {
    if (connections[i].server_id.matches(pub_key)) {
      connections[i].keep_alive_millis = 0;  // mark slot as now free
      connections[i].next_ping = 0;
      connections[i].expected_ack = 0;
      connections[i].last_activity = 0;
      break;
    }
  }
}

bool BaseChatMesh::hasConnectionTo(const uint8_t* pub_key) {
  for (int i = 0; i < MAX_CONNECTIONS; i++) {
    if (connections[i].keep_alive_millis > 0 && connections[i].server_id.matches(pub_key)) return true;
  }
  return false;
}

void BaseChatMesh::markConnectionActive(const ContactInfo& contact) {
  for (int i = 0; i < MAX_CONNECTIONS; i++) {
    if (connections[i].keep_alive_millis > 0 && connections[i].server_id.matches(contact.id)) {
      connections[i].last_activity = getRTCClock()->getCurrentTime();

      // re-schedule next KEEP_ALIVE, now that we have heard from server
      connections[i].next_ping = futureMillis(connections[i].keep_alive_millis);
      break;
    }
  }
}

ContactInfo* BaseChatMesh::checkConnectionsAck(const uint8_t* data) {
  for (int i = 0; i < MAX_CONNECTIONS; i++) {
    if (connections[i].keep_alive_millis > 0 && memcmp(&connections[i].expected_ack, data, 4) == 0) {
      // yes, got an ack for our keep_alive request!
      connections[i].expected_ack = 0;
      connections[i].last_activity = getRTCClock()->getCurrentTime();

      // re-schedule next KEEP_ALIVE, now that we have heard from server
      connections[i].next_ping = futureMillis(connections[i].keep_alive_millis);

      auto id = &connections[i].server_id;
      return lookupContactByPubKey(id->pub_key, PUB_KEY_SIZE);  // yes, a match
    }
  }
  return NULL;  /// no match
}

void BaseChatMesh::checkConnections() {
  // scan connections[] table, send KEEP_ALIVE requests
  for (int i = 0; i < MAX_CONNECTIONS; i++) {
    if (connections[i].keep_alive_millis == 0) continue;  // unused slot

    uint32_t now = getRTCClock()->getCurrentTime();
    uint32_t expire_secs = (connections[i].keep_alive_millis / 1000) * 5 / 2;   // 2.5 x keep_alive interval
    if (now >= connections[i].last_activity + expire_secs) {
      // connection now lost
      connections[i].keep_alive_millis = 0;
      connections[i].next_ping = 0;
      connections[i].expected_ack = 0;
      connections[i].last_activity = 0;
      continue;
    }

    if (millisHasNowPassed(connections[i].next_ping)) {
      auto contact = lookupContactByPubKey(connections[i].server_id.pub_key, PUB_KEY_SIZE);
      if (contact == NULL) {
        MESH_DEBUG_PRINTLN("checkConnections(): Keep_alive contact not found!");
        continue;
      }
      if (contact->out_path_len == OUT_PATH_UNKNOWN) {
        MESH_DEBUG_PRINTLN("checkConnections(): Keep_alive contact, no out_path!");
        continue;
      }

      // send KEEP_ALIVE request
      uint8_t data[9];
      uint32_t now = getRTCClock()->getCurrentTimeUnique();
      memcpy(data, &now, 4);
      data[4] = REQ_TYPE_KEEP_ALIVE;
      memcpy(&data[5], &contact->sync_since, 4);
    
      // calc expected ACK reply
      mesh::Utils::sha256((uint8_t *)&connections[i].expected_ack, 4, data, 9, self_id.pub_key, PUB_KEY_SIZE);

      auto pkt = createDatagram(PAYLOAD_TYPE_REQ, contact->id, contact->getSharedSecret(self_id), data, 9);
      if (pkt) {
        sendDirect(pkt, contact->out_path, contact->out_path_len);
      }
    
      // schedule next KEEP_ALIVE
      connections[i].next_ping = futureMillis(connections[i].keep_alive_millis);
    }
  }
}

void BaseChatMesh::resetPathTo(ContactInfo& recipient) {
  recipient.out_path_len = OUT_PATH_UNKNOWN;
}

static ContactInfo* table;  // pass via global :-(

static int cmp_adv_timestamp(const void *a, const void *b) {
  int a_idx = *((int *)a);
  int b_idx = *((int *)b);
  if (table[b_idx].last_advert_timestamp > table[a_idx].last_advert_timestamp) return 1;
  if (table[b_idx].last_advert_timestamp < table[a_idx].last_advert_timestamp) return -1;
  return 0;
}

void BaseChatMesh::scanRecentContacts(int last_n, ContactVisitor* visitor) {
  for (int i = 0; i < num_contacts; i++) {  // sort the INDEXES into contacts[]
    sort_array[i] = i;
  }
  table = contacts; // pass via global *sigh* :-(
  qsort(sort_array, num_contacts, sizeof(sort_array[0]), cmp_adv_timestamp);

  if (last_n == 0) {
    last_n = num_contacts;   // scan ALL
  } else {
    if (last_n > num_contacts) last_n = num_contacts;
  }
  for (int i = 0; i < last_n; i++) {
    visitor->onContactVisit(contacts[sort_array[i]]);
  }
}

ContactInfo* BaseChatMesh::searchContactsByPrefix(const char* name_prefix) {
  int len = strlen(name_prefix);
  for (int i = 0; i < num_contacts; i++) {
    auto c = &contacts[i];
    if (memcmp(c->name, name_prefix, len) == 0) return c;
  }
  return NULL;  // not found
}

ContactInfo* BaseChatMesh::lookupContactByPubKey(const uint8_t* pub_key, int prefix_len) {
  for (int i = 0; i < num_contacts; i++) {
    auto c = &contacts[i];
    if (memcmp(c->id.pub_key, pub_key, prefix_len) == 0) return c;
  }
  return NULL;  // not found
}

bool BaseChatMesh::addContact(const ContactInfo& contact) {
  ContactInfo* dest = allocateContactSlot(contact.type == ADV_TYPE_NONE);
  if (dest) {
    *dest = contact;
    dest->shared_secret_valid = false; // mark shared_secret as needing calculation
    return true;  // success
  }
  return false;
}

bool BaseChatMesh::removeContact(ContactInfo& contact) {
  int idx = 0;
  while (idx < num_contacts && !contacts[idx].id.matches(contact.id)) {
    idx++;
  }
  if (idx >= num_contacts) return false;   // not found

  // remove from contacts array
  num_contacts--;
  while (idx < num_contacts) {
    contacts[idx] = contacts[idx + 1];
    idx++;
  }
  return true;  // Success
}

#ifdef MAX_GROUP_CHANNELS
#include <base64.hpp>

ChannelDetails* BaseChatMesh::addChannel(const char* name, const char* psk_base64) {
  if (num_channels < MAX_GROUP_CHANNELS) {
    auto dest = &channels[num_channels];

    memset(dest->channel.secret, 0, sizeof(dest->channel.secret));
    int len = decode_base64((unsigned char *) psk_base64, strlen(psk_base64), dest->channel.secret);
    if (len == 32 || len == 16) {
      mesh::Utils::sha256(dest->channel.hash, sizeof(dest->channel.hash), dest->channel.secret, len);
      StrHelper::strncpy(dest->name, name, sizeof(dest->name));
      num_channels++;
      return dest;
    }
  }
  return NULL;
}
bool BaseChatMesh::getChannel(int idx, ChannelDetails& dest) {
  if (idx >= 0 && idx < MAX_GROUP_CHANNELS) {
    dest = channels[idx];
    return true;
  }
  return false;
}
bool BaseChatMesh::setChannel(int idx, const ChannelDetails& src) {
  static uint8_t zeroes[] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };

  if (idx >= 0 && idx < MAX_GROUP_CHANNELS) {
    channels[idx] = src;
    if (memcmp(&src.channel.secret[16], zeroes, 16) == 0) {
      mesh::Utils::sha256(channels[idx].channel.hash, sizeof(channels[idx].channel.hash), src.channel.secret, 16);  // 128-bit key
    } else {
      mesh::Utils::sha256(channels[idx].channel.hash, sizeof(channels[idx].channel.hash), src.channel.secret, 32);  // 256-bit key
    }
    return true;
  }
  return false;
}
int BaseChatMesh::findChannelIdx(const mesh::GroupChannel& ch) {
  for (int i = 0; i < MAX_GROUP_CHANNELS; i++) {
    if (memcmp(ch.secret, channels[i].channel.secret, sizeof(ch.secret)) == 0) return i;
  }
  return -1;  // not found
}
#else
ChannelDetails* BaseChatMesh::addChannel(const char* name, const char* psk_base64) {
  return NULL;  // not supported
}
bool BaseChatMesh::getChannel(int idx, ChannelDetails& dest) {
  return false;
}
bool BaseChatMesh::setChannel(int idx, const ChannelDetails& src) {
  return false;
}
int BaseChatMesh::findChannelIdx(const mesh::GroupChannel& ch) {
  return -1;  // not found
}
#endif

bool BaseChatMesh::getContactByIdx(uint32_t idx, ContactInfo& contact) {
  if (idx >= num_contacts) return false;

  contact = contacts[idx];
  return true;
}

ContactsIterator BaseChatMesh::startContactsIterator() {
  return ContactsIterator();
}

bool ContactsIterator::hasNext(const BaseChatMesh* mesh, ContactInfo& dest) {
  if (next_idx >= mesh->getNumContacts()) return false;

  dest = mesh->contacts[next_idx++];
  return true;
}

void BaseChatMesh::loop() {
  Mesh::loop();

  if (txt_send_timeout && millisHasNowPassed(txt_send_timeout)) {
    // failed to get an ACK
    onSendTimeout();
    txt_send_timeout = 0;
  }

  if (_pendingLoopback) {
    onRecvPacket(_pendingLoopback);  // loop-back, as if received over radio
    releasePacket(_pendingLoopback);   // undo the obtainNewPacket()
    _pendingLoopback = NULL;
  }
}
