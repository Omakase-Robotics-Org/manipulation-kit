from http import HTTPStatus
from typing import Any

import httpx

from ... import errors
from ...client import AuthenticatedClient, Client
from ...models.chassis_mapping_streams_response_200 import (
    ChassisMappingStreamsResponse200,
)
from ...models.error_envelope import ErrorEnvelope
from ...types import UNSET, Response, Unset


def _get_kwargs(
    *,
    stream_id: str | Unset = UNSET,
    map_revision: int | Unset = UNSET,
    map_modify_revision: int | Unset = UNSET,
    scan_revision: int | Unset = UNSET,
) -> dict[str, Any]:

    params: dict[str, Any] = {}

    params["stream_id"] = stream_id

    params["map_revision"] = map_revision

    params["map_modify_revision"] = map_modify_revision

    params["scan_revision"] = scan_revision

    params = {k: v for k, v in params.items() if v is not UNSET and v is not None}

    _kwargs: dict[str, Any] = {
        "method": "get",
        "url": "/v1/chassis/ros/mapping_streams",
        "params": params,
    }

    return _kwargs


def _parse_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> ChassisMappingStreamsResponse200 | ErrorEnvelope | None:
    if response.status_code == 200:
        response_200 = ChassisMappingStreamsResponse200.from_dict(response.json())

        return response_200

    if response.status_code == 400:
        response_400 = ErrorEnvelope.from_dict(response.json())

        return response_400

    if response.status_code == 404:
        response_404 = ErrorEnvelope.from_dict(response.json())

        return response_404

    if response.status_code == 409:
        response_409 = ErrorEnvelope.from_dict(response.json())

        return response_409

    if response.status_code == 502:
        response_502 = ErrorEnvelope.from_dict(response.json())

        return response_502

    if response.status_code == 504:
        response_504 = ErrorEnvelope.from_dict(response.json())

        return response_504

    if client.raise_on_unexpected_status:
        raise errors.UnexpectedStatus(response.status_code, response.content)
    else:
        return None


def _build_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> Response[ChassisMappingStreamsResponse200 | ErrorEnvelope]:
    return Response(
        status_code=HTTPStatus(response.status_code),
        content=response.content,
        headers=response.headers,
        parsed=_parse_response(client=client, response=response),
    )


def sync_detailed(
    *,
    client: AuthenticatedClient | Client,
    stream_id: str | Unset = UNSET,
    map_revision: int | Unset = UNSET,
    map_modify_revision: int | Unset = UNSET,
    scan_revision: int | Unset = UNSET,
) -> Response[ChassisMappingStreamsResponse200 | ErrorEnvelope]:
    """Read bounded live map and scan streams

     Separate read-only vendor ROS stream. Two occupancy grids are reported, because the base publishes
    its map under two different names in two different modes: `map` is the SLAM node's live grid,
    published only while the base is building a map, and `map_modify` is the decision node's copy of the
    loaded scene's grid, published latched once per scene load. `map_source` names which of the two is
    authoritative for the base's current decision work mode, and is `unknown` when no fresh mode was
    available. Outside mapping mode `map.status` is `not_published_in_this_mode`: nothing advertises the
    topic then, so its silence is the mode rather than a fault or a subscription that has not delivered
    yet. Because `map_modify` arrives once per scene load, a large `map_modify.age_ms` is normal and its
    staleness does not mean the map changed. Optional map_revision, map_modify_revision and
    scan_revision unsigned query parameters omit unchanged payloads while returning current
    validity/freshness metadata. Source timestamp is distinct from receipt freshness; latched receipt
    never proves mapping is active. Scan remains in its own frame; no TF overlay. Limits are application
    policies, not vendor maxima.

    Args:
        stream_id (str | Unset):
        map_revision (int | Unset):
        map_modify_revision (int | Unset):
        scan_revision (int | Unset):

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ChassisMappingStreamsResponse200 | ErrorEnvelope]
    """

    kwargs = _get_kwargs(
        stream_id=stream_id,
        map_revision=map_revision,
        map_modify_revision=map_modify_revision,
        scan_revision=scan_revision,
    )

    response = client.get_httpx_client().request(
        **kwargs,
    )

    return _build_response(client=client, response=response)


def sync(
    *,
    client: AuthenticatedClient | Client,
    stream_id: str | Unset = UNSET,
    map_revision: int | Unset = UNSET,
    map_modify_revision: int | Unset = UNSET,
    scan_revision: int | Unset = UNSET,
) -> ChassisMappingStreamsResponse200 | ErrorEnvelope | None:
    """Read bounded live map and scan streams

     Separate read-only vendor ROS stream. Two occupancy grids are reported, because the base publishes
    its map under two different names in two different modes: `map` is the SLAM node's live grid,
    published only while the base is building a map, and `map_modify` is the decision node's copy of the
    loaded scene's grid, published latched once per scene load. `map_source` names which of the two is
    authoritative for the base's current decision work mode, and is `unknown` when no fresh mode was
    available. Outside mapping mode `map.status` is `not_published_in_this_mode`: nothing advertises the
    topic then, so its silence is the mode rather than a fault or a subscription that has not delivered
    yet. Because `map_modify` arrives once per scene load, a large `map_modify.age_ms` is normal and its
    staleness does not mean the map changed. Optional map_revision, map_modify_revision and
    scan_revision unsigned query parameters omit unchanged payloads while returning current
    validity/freshness metadata. Source timestamp is distinct from receipt freshness; latched receipt
    never proves mapping is active. Scan remains in its own frame; no TF overlay. Limits are application
    policies, not vendor maxima.

    Args:
        stream_id (str | Unset):
        map_revision (int | Unset):
        map_modify_revision (int | Unset):
        scan_revision (int | Unset):

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ChassisMappingStreamsResponse200 | ErrorEnvelope
    """

    return sync_detailed(
        client=client,
        stream_id=stream_id,
        map_revision=map_revision,
        map_modify_revision=map_modify_revision,
        scan_revision=scan_revision,
    ).parsed


async def asyncio_detailed(
    *,
    client: AuthenticatedClient | Client,
    stream_id: str | Unset = UNSET,
    map_revision: int | Unset = UNSET,
    map_modify_revision: int | Unset = UNSET,
    scan_revision: int | Unset = UNSET,
) -> Response[ChassisMappingStreamsResponse200 | ErrorEnvelope]:
    """Read bounded live map and scan streams

     Separate read-only vendor ROS stream. Two occupancy grids are reported, because the base publishes
    its map under two different names in two different modes: `map` is the SLAM node's live grid,
    published only while the base is building a map, and `map_modify` is the decision node's copy of the
    loaded scene's grid, published latched once per scene load. `map_source` names which of the two is
    authoritative for the base's current decision work mode, and is `unknown` when no fresh mode was
    available. Outside mapping mode `map.status` is `not_published_in_this_mode`: nothing advertises the
    topic then, so its silence is the mode rather than a fault or a subscription that has not delivered
    yet. Because `map_modify` arrives once per scene load, a large `map_modify.age_ms` is normal and its
    staleness does not mean the map changed. Optional map_revision, map_modify_revision and
    scan_revision unsigned query parameters omit unchanged payloads while returning current
    validity/freshness metadata. Source timestamp is distinct from receipt freshness; latched receipt
    never proves mapping is active. Scan remains in its own frame; no TF overlay. Limits are application
    policies, not vendor maxima.

    Args:
        stream_id (str | Unset):
        map_revision (int | Unset):
        map_modify_revision (int | Unset):
        scan_revision (int | Unset):

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ChassisMappingStreamsResponse200 | ErrorEnvelope]
    """

    kwargs = _get_kwargs(
        stream_id=stream_id,
        map_revision=map_revision,
        map_modify_revision=map_modify_revision,
        scan_revision=scan_revision,
    )

    response = await client.get_async_httpx_client().request(**kwargs)

    return _build_response(client=client, response=response)


async def asyncio(
    *,
    client: AuthenticatedClient | Client,
    stream_id: str | Unset = UNSET,
    map_revision: int | Unset = UNSET,
    map_modify_revision: int | Unset = UNSET,
    scan_revision: int | Unset = UNSET,
) -> ChassisMappingStreamsResponse200 | ErrorEnvelope | None:
    """Read bounded live map and scan streams

     Separate read-only vendor ROS stream. Two occupancy grids are reported, because the base publishes
    its map under two different names in two different modes: `map` is the SLAM node's live grid,
    published only while the base is building a map, and `map_modify` is the decision node's copy of the
    loaded scene's grid, published latched once per scene load. `map_source` names which of the two is
    authoritative for the base's current decision work mode, and is `unknown` when no fresh mode was
    available. Outside mapping mode `map.status` is `not_published_in_this_mode`: nothing advertises the
    topic then, so its silence is the mode rather than a fault or a subscription that has not delivered
    yet. Because `map_modify` arrives once per scene load, a large `map_modify.age_ms` is normal and its
    staleness does not mean the map changed. Optional map_revision, map_modify_revision and
    scan_revision unsigned query parameters omit unchanged payloads while returning current
    validity/freshness metadata. Source timestamp is distinct from receipt freshness; latched receipt
    never proves mapping is active. Scan remains in its own frame; no TF overlay. Limits are application
    policies, not vendor maxima.

    Args:
        stream_id (str | Unset):
        map_revision (int | Unset):
        map_modify_revision (int | Unset):
        scan_revision (int | Unset):

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ChassisMappingStreamsResponse200 | ErrorEnvelope
    """

    return (
        await asyncio_detailed(
            client=client,
            stream_id=stream_id,
            map_revision=map_revision,
            map_modify_revision=map_modify_revision,
            scan_revision=scan_revision,
        )
    ).parsed
